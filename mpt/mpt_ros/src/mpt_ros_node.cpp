#include <cassert>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sstream>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <fcl/fcl.h>
#include <mpt/discrete_motion_validator.hpp>
#include <mpt/goal_state.hpp>
#include <mpt/se3_space.hpp>
#include <mpt/uniform_sampler.hpp>
#include <mpt/prrt_star.hpp>
#include <nigh/kdtree_batch.hpp>
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "mpt_ros/msg/motion_plan_request.hpp"
#include <iostream>
using std::placeholders::_1;

using Clock = std::chrono::steady_clock;

namespace mpt_ros {
    using namespace unc::robotics;

    template <class S>
    using Mesh = fcl::BVHModel<fcl::OBBRSS<S>>;

    // template <class S>
    // auto mapToEigen(const aiMatrix4x4t<S>& m) {
    //     using Matrix = const Eigen::Matrix<S, 4, 4, Eigen::RowMajor>;
    //     static_assert(sizeof(Matrix) == sizeof(m));
    //     return Eigen::Map<Matrix>(&m.a1);
    // }

    template <class S>
    auto mapToEigen(const aiVector3t<S>& v) {
        using Vector = const Eigen::Matrix<S, 3, 1>;
        static_assert(sizeof(Vector) == sizeof(v));
        return Eigen::Map<Vector>(&v.x);
    }
    
    // template <class S> // , int mode>
    std::pair<aiVector3D, std::size_t>
    computeMeshCenter(
        const aiScene *scene, const aiNode *node,
        aiMatrix4x4 transform)
    // Eigen::Transform<S, 3, mode> transform)
    {
        // Eigen::Matrix<S, 3, 1> center = Eigen::Matrix<S, 3, 1>::Zero();
        aiVector3D center;
        center.Set(0,0,0);
        std::size_t count = 0;
        //transform *= mapToEigen(node->mTransformation).template cast<S>();
        transform *= node->mTransformation;
        for (unsigned i=0 ; i<node->mNumMeshes ; ++i) {
            const aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
            count += mesh->mNumVertices;
            for (unsigned j=0 ; j<mesh->mNumVertices ; ++j)
                //center += transform * mapToEigen(mesh->mVertices[j]).template cast<S>();
                center += transform * mesh->mVertices[j];
        }
        for (unsigned i=0 ; i<node->mNumChildren ; ++i) {
            auto [childCenter, childCount ] = computeMeshCenter(scene, node->mChildren[i], transform);
            center += childCenter;
            count += childCount;
        }
        return { center, count };
    }

    template <class S>
    void addNodeToMesh(const aiScene *scene, const aiNode *node, aiMatrix4x4 transform, Mesh<S>& model) {
        using Vec3 = Eigen::Matrix<S, 3, 1>;
        transform *= node->mTransformation;
        // MPT_LOG(INFO) << "  transform = " << transform.matrix();
        for (unsigned i=0 ; i<node->mNumMeshes ; ++i) {
            const aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
            for (unsigned j=0 ; j<mesh->mNumFaces ; ++j) {
                const aiFace& face = mesh->mFaces[j];
                if (face.mNumIndices < 3)
                    continue;

                // Support trangular decomposition by fanning out
                // around vertex 0.  The indexing follows as:
                //
                //   0---1   0 1 2
                //  /|\ /    0 2 3
                // 4-3-2     0 3 4
                //
                aiVector3D v0 = transform * mesh->mVertices[face.mIndices[0]];
                aiVector3D v1 = transform * mesh->mVertices[face.mIndices[1]];
                for (unsigned k=2 ; k<face.mNumIndices ; ++k) {
                    aiVector3D v2 = transform * mesh->mVertices[face.mIndices[k]];
                    model.addTriangle(
                        mapToEigen(v0).template cast<S>(),
                        mapToEigen(v1).template cast<S>(),
                        mapToEigen(v2).template cast<S>());
                    v1 = v2;
                }
            }
        }
        for (unsigned i=0 ; i<node->mNumChildren ; ++i)
            addNodeToMesh(scene, node->mChildren[i], transform, model);
    }
    
    template <class S>
    std::shared_ptr<Mesh<S>> loadMesh(
        const void *pBuffer, std::size_t pLength, const char *pHint, bool shiftToCenter)
    {
        using Transform = fcl::Transform3<S>;
        Assimp::Importer importer;

        static constexpr auto readOpts
            = aiProcess_Triangulate
            | aiProcess_JoinIdenticalVertices
            | aiProcess_SortByPType
            | aiProcess_OptimizeGraph
            | aiProcess_GenNormals
            //| aiProcess_OptimizeMeshes
            ;

        const aiScene *scene = importer.ReadFileFromMemory(
            pBuffer, pLength, readOpts, pHint);

        if (scene == nullptr)
            throw std::invalid_argument("could not load mesh");

        if (!scene->HasMeshes())
            throw std::invalid_argument("no meshes in data");

        // Transform rootTransform = Transform::Identity();
        aiMatrix4x4 rootTransform;

        if (shiftToCenter) {
            // auto [ sum, count ] = computeMeshCenter(scene, scene->mRootNode, Transform::Identity());
            auto [ aiSum, count ] = computeMeshCenter(scene, scene->mRootNode, aiMatrix4x4());
            Eigen::Matrix<S, 3, 1> sum = mapToEigen(aiSum).template cast<S>();
            MPT_LOG(INFO) << "Shift to center " << sum << " / " << count << " = " << (sum/count);
            //rootTransform *= Eigen::Translation<S, 3>(sum / -count);
            aiSum *= -1.0/count;
            aiMatrix4x4::Translation(aiSum, rootTransform);
        }

        std::shared_ptr<Mesh<S>> mesh = std::make_shared<Mesh<S>>();
        mesh->beginModel();
        addNodeToMesh(scene, scene->mRootNode, rootTransform, *mesh);
        mesh->endModel();
        mesh->computeLocalAABB();
        return mesh;
    }

    template <class S>
    class SE3RigidBodyScenario {
    public:
        static constexpr std::intmax_t SO3_WEIGHT = 50; // weight SO3 by 50 relative to translation
        using Space = mpt::SE3Space<S, SO3_WEIGHT>; 
        using Bounds = std::tuple<mpt::Unbounded, mpt::BoxBounds<S, 3>>;
        using State = typename Space::Type;
        using Distance = typename Space::Distance;
        using Goal = mpt::GoalState<Space>;
        using Nearest = nigh::KDTreeBatch<8>;

    private:
        using Config = typename Space::Type;
        using Transform = fcl::Transform3<S>;

        std::shared_ptr<Mesh<S>> environment_;
        std::shared_ptr<Mesh<S>> robot_;

        Space space_;
        Bounds bounds_;

        static constexpr Distance goalRadius = 1e-6;
        Goal goal_;

        S checkResolution_;

        struct Validator {
            const SE3RigidBodyScenario& scenario_;
            
            Validator(const SE3RigidBodyScenario& scenario)
                : scenario_(scenario)
            {
            }
            
            bool operator () (const Config& q) const {
                return scenario_.valid(q);
            }
        };
        
        struct ReportingValidator {
            const SE3RigidBodyScenario& scenario_;
            mutable int count_{0};
            
            ReportingValidator(const SE3RigidBodyScenario& scenario)
                : scenario_(scenario)
            {
            }

            ~ReportingValidator() {
                if (count_)
                    MPT_LOG(DEBUG) << "link count " << count_;
            }
            
            bool operator () (const Config& q) const {
                ++count_;
                //MPT_LOG(INFO) << q;
                return scenario_.valid(q);
            }
        };

        // mpt::DiscreteMotionValidator<Space, Validator> link_;
        
    public:
        template <class Min, class Max>
        SE3RigidBodyScenario(
            const std::shared_ptr<Mesh<S>>& envMesh,
            const std::shared_ptr<Mesh<S>>& robotMesh,
            const Config& goal,
            const Eigen::MatrixBase<Min>& min,
            const Eigen::MatrixBase<Max>& max,
            S checkResolution)
            : environment_(envMesh)
            , robot_(robotMesh)
            , bounds_(mpt::Unbounded{}, mpt::BoxBounds<S, 3>(min, max))
            , goal_(goalRadius, goal)
            , checkResolution_(checkResolution)
              //, link_(space_, ((max - min).norm() - S(SO3_WEIGHT*3.1415/2))*checkResolution, Validator(*this))
        {
            assert(environment_);
            assert(robot_);

            MPT_LOG(INFO) << "World = " << environment_->num_tris << ", " << environment_->num_vertices;
            MPT_LOG(INFO) << "Robot = " << robot_->num_tris << ", " << robot_->num_vertices;
            MPT_LOG(INFO) << "Min = " << std::get<1>(bounds_).min();
            MPT_LOG(INFO) << "Max = " << std::get<1>(bounds_).max();
            MPT_LOG(INFO) << "Goal  = " << goal;
        }

    private:
        static Transform stateToTransform(const Config& q) {
            Transform t = Transform::Identity();
            t.linear() = std::get<Eigen::Quaternion<S>>(q).toRotationMatrix();
            t.translation() = std::get<Eigen::Matrix<S,3,1>>(q);
            return t;
            // return Eigen::Translation<S, 3>(std::get<Eigen::Matrix<S,3,1>>(q))
            //     * std::get<Eigen::Quaternion<S>>(q);
        }

    public:
        const Space& space() const {
            return space_;
        }

        const Bounds& bounds() const {
            return bounds_;
        }

        const Goal& goal() const {
            return goal_;
        }

        bool valid(const Config& q) const {
            fcl::CollisionRequest<S> req;
            fcl::CollisionResult<S> res;

            Transform tf = stateToTransform(q);
            return 0 == fcl::collide(
                environment_.get(), Transform::Identity(),
                robot_.get(), tf,
                req, res);
        }

        bool link(const Config& a, const Config& b) const {
            mpt::DiscreteMotionValidator<Space, Validator> link_(
                space_,
                ((std::get<1>(bounds_).min() - std::get<1>(bounds_).max()).norm() - S(SO3_WEIGHT*3.1415/2))*checkResolution_,
                Validator(*this));
            return link_(a, b);
        }

        bool linkReport(const Config& a, const Config& b) const {
            mpt::DiscreteMotionValidator<Space, ReportingValidator> link_(
                space_,
                ((std::get<1>(bounds_).min() - std::get<1>(bounds_).max()).norm() - S(SO3_WEIGHT*3.1415/2))*checkResolution_,
                ReportingValidator(*this));
            return link_(a, b);
        }
    };

    using namespace unc::robotics;


    template <class S>
    class MPTNode : public rclcpp::Node {
        using Scenario = SE3RigidBodyScenario<S>;
        using State = typename Scenario::State;
        using Mesh = mpt_ros::Mesh<S>;
        
        rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr envMeshSub_;
        rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr robotMeshSub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr boundsSub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr planStartToGoalSub_;
        rclcpp::Subscription<mpt_ros::msg::MotionPlanRequest>::SharedPtr motionPlanRequestSub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motionPlanPub_;

        struct MotionPlanRequest {
            std::shared_ptr<Mesh> environment_;
            std::shared_ptr<Mesh> robot_;
            Eigen::Matrix<S, 3, 1> vMin_;
            Eigen::Matrix<S, 3, 1> vMax_;
            State qStart_;
            State qGoal_;
            double minPlanningTime_ = 0;
            double maxPlanningTime_ = 300;
            double minSolutionCost_ = 0;
            
            MotionPlanRequest() {
            }
            
            MotionPlanRequest(
                const std::shared_ptr<Mesh>& env,
                const std::shared_ptr<Mesh>& robot,
                const State& qStart,
                const State& qGoal)
                : environment_(env)
                , robot_(robot)
                , qStart_(qStart)
                , qGoal_(qGoal)
            {
            }
        };

        MotionPlanRequest motionPlanRequest_;
        bool planHasBounds_{false};
        bool planHasStartAndGoal_{false};

        // We implemente a 1-item queue between callbacks and planning
        // thread based on a std::optional.  We also overwrite this
        // queue entry with the latest request so that when the
        // planning thread is ready, it will run the most recent plan.
        std::mutex runStateMutex_;
        std::atomic<bool> shutdown_{false};
        std::optional<MotionPlanRequest> nextRequest_;
        std::condition_variable nextRequestReady_;

        std::thread planThread_;

        template <class Algorithm>
        void runPlanner(
            const Scenario& scenario, const typename Scenario::State& qStart,
            double minPlanningTime, double maxPlanningTime,
            double minSolutionCost)
        {
            mpt::Planner<Scenario, Algorithm> planner(scenario);
            planner.addStart(qStart);

            using Seconds = std::chrono::duration<double>;
            auto startTime = Clock::now();
            using namespace std::literals;
            planner.solve(
                [&] () {
                    // return shutdown_ || planner.solved();
                    if (shutdown_)
                        return true;
                    double elapsedTime = Seconds(Clock::now() - startTime).count();
                    if (elapsedTime < minPlanningTime)
                        return false;
                    if (planner.solved() && planner.solutionCost() <= minSolutionCost)
                        return true;
                    if (elapsedTime > maxPlanningTime)
                        return true;

                    return false;
                });
            // planner.solveFor(10s);
            auto elapsed = Clock::now() - startTime;

            RCLCPP_INFO(this->get_logger(), "Planner ran for %lf seconds", Seconds(elapsed).count());

            std_msgs::msg::Float64MultiArray msg;
            msg.layout.data_offset = 0;
            msg.layout.dim.emplace_back();
            if (!planner.solved()) {
                msg.layout.dim[0].label = "failed";
                msg.layout.dim[0].size = 0;
                msg.layout.dim[0].stride = 7;
            } else {
                std::vector<State> path = planner.solution();
                // auto it = path.begin();
                // if (it != path.end()) {
                //     for (auto prev = it ; ++it != path.end() ; prev = it) {
                //         MPT_LOG(INFO) << "Valid link: " << scenario.linkReport(*prev, *it);
                //     }
                // }
                
                msg.layout.dim.emplace_back();
                msg.layout.dim[0].label = "plan";
                msg.layout.dim[0].size = path.size();
                msg.layout.dim[0].stride = 7;
                msg.layout.dim[1].label = "waypoint";
                msg.layout.dim[1].size = 7;
                msg.data.reserve(path.size() * 7);
                for (std::size_t i=0 ; i<path.size() ; ++i) {
                    for (int j=0 ; j<4 ; ++j)
                        msg.data.push_back(std::get<Eigen::Quaternion<S>>(path[i]).coeffs()[j]);
                    for (int j=0 ; j<3 ; ++j)
                        msg.data.push_back(std::get<Eigen::Matrix<S, 3, 1>>(path[i])[j]);
                }
            }

            motionPlanPub_->publish(msg);
        }

        void plannerLoop() {
            // using Algorithm = mpt::PRRTStar<nigh::KDTreeBatch<>, mpt::single_threaded>;
            using Algorithm = mpt::PRRTStar<nigh::KDTreeBatch<>, mpt::hardware_concurrency>;

            RCLCPP_INFO(this->get_logger(), "Planner loop starting");
                            
            std::unique_lock<std::mutex> lock(runStateMutex_);
            while (!shutdown_) {
                nextRequestReady_.wait(lock);
                RCLCPP_INFO(this->get_logger(), "Planner loop awake");
                if (nextRequest_) {
                    S checkResolution = 1e-4;
                    Scenario scenario(
                        std::move(nextRequest_->environment_),
                        std::move(nextRequest_->robot_),
                        nextRequest_->qGoal_,
                        nextRequest_->vMin_,
                        nextRequest_->vMax_,
                        checkResolution);
                    double minPlanningTime = nextRequest_->minPlanningTime_;
                    double maxPlanningTime = nextRequest_->maxPlanningTime_;
                    double minSolutionCost = nextRequest_->minSolutionCost_;
                    State qStart = nextRequest_->qStart_;
                    nextRequest_.reset();

                    lock.unlock();
                    runPlanner<Algorithm>(scenario, qStart, minPlanningTime, maxPlanningTime, minSolutionCost);
                    lock.lock();
                }
            }
        }
    
        auto meshCallback(const std::string& type, const std_msgs::msg::UInt8MultiArray& msg, bool shiftToCenter) {
            RCLCPP_INFO(this->get_logger(), "Got %s, %d, %d, %s", type.c_str(), (int)msg.data.size(), (int)msg.layout.dim.size(),
                     msg.layout.dim.size() ? msg.layout.dim[0].label.c_str() : "(none)");
            //msg.layout.size()
            auto startLoad = Clock::now();
            auto mesh = mpt_ros::loadMesh<double>(
                msg.data.data(), msg.data.size(), msg.layout.dim[0].label.c_str(), shiftToCenter);
            auto elapsed = Clock::now() - startLoad;
            RCLCPP_INFO(this->get_logger(), "Loaded mesh in %lf seconds, tris=%d, verts=%d",
                     std::chrono::duration<double>(elapsed).count(),
                     mesh->num_tris, mesh->num_vertices);
            return mesh;
        }

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> loadMatrix(
            const std::string& name, const std_msgs::msg::Float64MultiArray& msg) 
        {
            int rows = 0;
            int cols = 0;
                
            if (msg.layout.dim.size() == 2) {
                rows = msg.layout.dim[0].size;
                cols = msg.layout.dim[1].size;
            }
            
            RCLCPP_INFO(this->get_logger(), "Got %s, %dx%d=%d", name.c_str(), rows, cols, (int)msg.data.size());
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m(rows, cols);
            for (int c=0, i=0 ; c<cols ; ++c)
                for (int r=0 ; r<rows ; ++r, ++i)
                    m(r, c) = msg.data[i];
            
            return m;
        }
        
        template <class Source>
        void convertToSE3State(
            const Eigen::MatrixBase<Source>& src,
            mpt_ros::SE3RigidBodyScenario<double>::State& dst)
        {
            std::get<Eigen::Matrix<double, 3, 1>>(dst) = src.template head<3>();
            std::get<Eigen::Quaternion<double>>(dst) = Eigen::AngleAxis<double>(
                src[3], src.template tail<3>());

            MPT_LOG(INFO) << "Got state: " << dst;
        }

        void optRunPlanner() {
            if (motionPlanRequest_.environment_ && motionPlanRequest_.robot_ &&
                planHasStartAndGoal_ && planHasBounds_)
            {
                planHasStartAndGoal_ = false;

                std::unique_lock<std::mutex> lock(runStateMutex_);
                bool notify = !nextRequest_;
                nextRequest_.emplace(motionPlanRequest_);
                lock.unlock();

                // only notify if there wasn't a plan already.
                if (notify)
                    nextRequestReady_.notify_one();
            }
        }
        
        void environmentMeshCallback(const std_msgs::msg::UInt8MultiArray& msg) {
            motionPlanRequest_.environment_ = meshCallback("environment", msg, false);
            optRunPlanner();
        }

        void robotMeshCallback(const std_msgs::msg::UInt8MultiArray& msg) {
            motionPlanRequest_.robot_ = meshCallback("robot", msg, true);
            optRunPlanner();
        }

        void environmentBoundsCallback(const std_msgs::msg::Float64MultiArray& msg) {
            auto bounds = loadMatrix("bounds", msg);
            if (bounds.rows() == 3 && bounds.cols() == 2) {
                motionPlanRequest_.vMin_ = bounds.col(0);
                motionPlanRequest_.vMax_ = bounds.col(1);
                planHasBounds_ = true;
                MPT_LOG(INFO) << "GOT Bounds\n\tmin=" << motionPlanRequest_.vMin_ << "\n\tmax=" << motionPlanRequest_.vMax_;
                optRunPlanner();
            }
        }

        void planStartToGoalCallback(const std_msgs::msg::Float64MultiArray& msg) {
            auto startAndGoal = loadMatrix("start and goal", msg);
            if (startAndGoal.rows() == 7 && startAndGoal.cols() == 2) {
                convertToSE3State(startAndGoal.col(0), motionPlanRequest_.qStart_);
                convertToSE3State(startAndGoal.col(1), motionPlanRequest_.qGoal_);
                planHasStartAndGoal_ = true;
                optRunPlanner();
            }
        }

        void convertToSE3State(
            const std::vector<double>& src,
            mpt_ros::SE3RigidBodyScenario<double>::State& dst)
        {
            for (int i=0 ; i<3 ; ++i)
                std::get<Eigen::Matrix<double, 3, 1>>(dst)[i] = src[i];
            Eigen::Matrix<double, 3, 1> axis;
            for (int i=0 ; i<3 ; ++i)
                axis[i] = src[4+i];
            std::get<Eigen::Quaternion<double>>(dst) = Eigen::AngleAxis<double>(
                src[3], axis);

            MPT_LOG(INFO) << "Got state " << dst;
        }

        void motionPlanRequestCallback(const mpt_ros::msg::MotionPlanRequest& msg) {
            RCLCPP_INFO(this->get_logger(), "Motion plan request");
            for (int i=0 ; i<3 ; ++i)
                motionPlanRequest_.vMin_[i] = msg.bounds_min[i];
            for (int i=0 ; i<3 ; ++i)
                motionPlanRequest_.vMax_[i] = msg.bounds_max[i];

            convertToSE3State(msg.start_config, motionPlanRequest_.qStart_);
            convertToSE3State(msg.goal_config, motionPlanRequest_.qGoal_);
            planHasStartAndGoal_ = true;
            planHasBounds_ = true;
            
            motionPlanRequest_.minPlanningTime_ = msg.min_planning_time;
            motionPlanRequest_.maxPlanningTime_ = msg.max_planning_time;
            motionPlanRequest_.minSolutionCost_ = msg.min_solution_cost;
            
            optRunPlanner();
        }

    public:
        
        MPTNode()
            : envMeshSub_(this->create_subscription<std_msgs::msg::UInt8MultiArray>("environment_mesh", 1000, std::bind(&MPTNode::environmentMeshCallback, this, _1)))
            , robotMeshSub_(this->create_subscription<std_msgs::msg::UInt8MultiArray>("robot_mesh", 1000, std::bind(&MPTNode::robotMeshCallback, this, _1)))
            , planStartToGoalSub_(this->create_subscription<std_msgs::msg::Float64MultiArray>("plan_start_to_goal", 1000, std::bind(&MPTNode::planStartToGoalCallback, this, _1)))
            , boundsSub_(this->create_subscription<std_msgs::msg::Float64MultiArray>("environment_bounds", 1000, std::bind(&MPTNode::environmentBoundsCallback, this, _1)))
            , motionPlanRequestSub_(this->create_subscription<mpt_ros::msg::MotionPlanRequest>("motion_plan_request", 1000, std::bind(&MPTNode::motionPlanRequestCallback, this, _1)))
            , motionPlanPub_(this->create_publisher<std_msgs::msg::Float64MultiArray>("motion_plan", 1000))
            , planThread_(&MPTNode::plannerLoop, this),
            Node("mpt_node")
        {
        }

        ~MPTNode() {
            // tODO: shutdown sequence (Set shutdown = true, wait for thread)
            shutdown_ = true;
            nextRequestReady_.notify_one();
            //std::unique_lock<std::mutex> lock(nextRequestReady_);
            planThread_.join();
        }
            
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mpt_ros::MPTNode<double>>();

    // ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    // ros::Subscriber envMeshSub = n.subscribe("environment_mesh", 1000, environmentMeshCallback);
    // ros::Subscriber robotMeshSub = n.subscribe("robot_mesh", 1000, robotMeshCallback);
    // ros::Subscriber boundsSub = n.subscribe("environment_bounds", 1000, environmentBoundsCallback);
    // ros::Subscriber planSub = n.subscribe("plan_start_to_goal", 1000, planStartToGoalCallback);

    RCLCPP_INFO(node->get_logger(), "Starting cloud");
    rclcpp::spin(node);
    return 0;
}

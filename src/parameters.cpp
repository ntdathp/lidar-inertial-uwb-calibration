#include "parameters.h"
#include "yaml-cpp/yaml.h"


bool mode_fine_opt;

int n_iter = 50,
    n_iter_each_opt = -1,
    n_splits,
    discard_nth_split,
    robust_loss_fn;

double  tag_ranging_observation_variance,
        tag_ranging_synth_variance;

double td;

double scale_res_a2a = 1.0;
double scale_res_tdoa = 1.0;
double scale_res_prior = 1.0;
double huber_loss_tdoa = 1.0;

Eigen::Vector3d L_t_U(0.0, 0.0, 0.0);

template <typename T>
T readLaunchParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readConfigFile(ros::NodeHandle &n) {
    std::string config_file;
    config_file = readLaunchParam<std::string>(n, "config_file");
    YAML::Node config = YAML::LoadFile(config_file);

    n_iter = config["n_iter"].as<int>();
    n_splits = config["n_splits"].as<int>();
    discard_nth_split = config["discard_nth_split"].as<int>();
    robust_loss_fn = config["robust_loss_fn"].as<int>();
    td = config["td"].as<double>();
    mode_fine_opt = config["mode_fine_opt"].as<bool>();
    
    scale_res_a2a = config["scale_res_a2a"].as<double>();
    scale_res_tdoa = config["scale_res_tdoa"].as<double>();
    scale_res_prior = config["scale_res_prior"].as<double>();

    huber_loss_tdoa = config["huber_loss_tdoa"].as<double>();

    ROS_INFO("parameters:");
    std::cout << "n_iter:                          \t" << n_iter << std::endl;
    std::cout << "n_splits:                        \t" << n_splits << std::endl;
    std::cout << "discard_nth_split:               \t" << discard_nth_split << std::endl;
    std::cout << "robust_loss_fn:                  \t" << robust_loss_fn << std::endl;
    std::cout << "td:                              \t" << td << std::endl;
    std::cout << "scale_res_a2a:                   \t" << scale_res_a2a << std::endl;
    std::cout << "scale_res_tdoa:                  \t" << scale_res_tdoa << std::endl;
    std::cout << "scale_res_prior:                 \t" << scale_res_prior << std::endl;
    std::cout << "huber_loss_tdoa:                 \t" << huber_loss_tdoa << std::endl;
    std::cout << "mode_fine_opt:                   \t" << mode_fine_opt << std::endl;
    // std::cout << "L_t_U:                             \t" << L_t_U.transpose() << std::endl;
}

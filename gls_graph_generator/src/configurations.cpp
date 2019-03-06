#include "configurations.hpp"


std::vector<Eigen::VectorXd> getSeedConfigurationsForFeeding()
{
  std::vector<Eigen::VectorXd> configurations;
  Eigen::VectorXd seedConfiguration(Eigen::VectorXd::Zero(6));

  seedConfiguration << 0.71149, 1.96881, 2.12461, -1.60078, -2.06181, -2.33079;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -5.34054, 1.9574, 1.86268, 0.0843705, -4.48063, 5.66238;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.3927, 4.3067, 3.94881, -4.62768, -2.25937, -2.07654;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.39079, 4.30507, 3.96979, 1.65921, -2.25655, -2.08021;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.46508, 4.35337, 3.95698, 1.76356, -2.28537, -1.9966;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.18807, 3.26458, 1.77104, -2.4695, -2.02196, -2.07969;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.14454, 3.23998, 1.60007, -2.57557, -1.81261, 2.99757;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.1928, 3.40455, 1.84249, -2.52313, -1.92402, 3.76996;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.32828, 4.19546, 3.72812, 1.71219, -2.42468, 4.38807;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.1928, 3.40455, 1.84249, -2.52313, -1.92402, -2.51322;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -1.4778, 2.92522, 1.00283, -2.08638,  1.44895,  1.32235;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.06047, 3.33506, 1.90762, -2.4133, -2.11661, -0.682215;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration <<-2.35264, 4.24509, 3.96006, 1.58616, -2.21484, -2.15233;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -1.47776, 2.92554, 1.00349, -2.08592, 1.44893, 1.3223;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -3.14042, 3.15514, 1.16615, 0.573695, 1.59742, -1.28865;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.40444, 3.31599, 1.88258, -0.724555, 2.11118, -1.53751;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << 0.753014, 1.9752, 1.89462, 0.238096, 1.83547, -0.636708;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.32925, 4.22822, 3.84025, 1.66769, -2.3464, -1.99578;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -1.99374, 3.30963, 1.8211, -2.46515, -2.03034, -2.25597;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.15136, 3.41949, 1.69291, -2.6289, -1.69653, -2.03035;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.36154, 4.24923, 3.86001, 1.69034, -2.33204, -1.9959;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.32705, 4.18856, 3.72915, 1.7198, -2.41678, 10.6595;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration <<-2.44866, 4.33757, 3.93095, 1.74906, -2.2925, -1.99095;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -0.90715, 4.10547, 4.81739, 1.75586, -2.27429, -2.01147 ;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.64253, 2.88948, 1.10437, 0.0433776, 1.1455, 2.04545;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.77921, 2.84293, 1.07646,  -0.0206763, 1.09828, 2.25931;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -5.39202, 1.94476, 1.82651, 0.14991, 1.78208, -0.637554;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.34241, 4.20543, 3.84994, 1.64344, -2.3003, -2.04766 ;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -1.99782, 3.35221, 1.84423, -2.48742, -1.99844, -1.91277;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -1.63667, 2.94542, 0.962171, 2.92917, -1.22056,  1.81286;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -1.81719,  2.82256,  1.56441,  2.92916, -1.22059, 1.81307;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << 0.546467, 1.96717, 1.72055, 0.503852, 1.73615, -0.751649;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -1.0798, 2.72718, 1.74161, -1.58974,  1.51289, 0.913531;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -1.9401, 3.92174, 3.38932, -3.75006, 2.59724, -0.817528;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -1.48081, 4.19045, 4.09752, 1.70698, -2.03137, -2.32713;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.13102, 3.54862, 1.99171, -2.52359, -1.92639, -2.06387;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -1.66161, 3.18774, 1.69401, 3.23128, -1.80354, -4.45043;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.11237, 3.29138, 1.93036, -2.3675, -2.23003, -0.79868;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.14079, 3.30573, 1.94499, -2.3777, -2.18687, -5.28631;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.12146, 3.94101, 3.34574, 2.99502, -3.92992, -0.440412;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -8.07629, 3.47886, 2.15987, -2.64209, -2.53331, 2.74152;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.12611, 3.30686, 2.03593, -2.27111, -2.34318, 3.92119;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.19111, 3.32611, 2.0867, -2.19583, -2.42988, 2.93824;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.1018, 3.44181, 1.90086, -2.50312, -1.95277, 6.93331;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << 0.816582, 2.88355, 3.96263, 1.86817, 2.83235, 4.47004;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -1.81795, 3.19223, 1.24379, 2.93106, -1.18668, 7.97186;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << 0.871881, 2.89292, 4.01451, 7.52685, -2.7644, 8.56144;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << 0.761478, 2.9263, 4.02089, 1.89851, 2.81683, 0.202515;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << 0.531097, 2.06645, 1.60537, 0.602812, 1.60736, -0.878579;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -5.5214, 2.92906, 4.00137, 1.90192,  2.81911, 0.205562;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.32382,  3.3865,  2.30347, 4.8859, 2.9721, -7.43382;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << 6.90501, 2.89118, 4.4486, 2.51436, 1.92913, -7.63507;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.30184, 4.24077, 3.87564, -4.67058, 3.94133, -8.32409;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -14.6813,  4.24396,  4.22699, 3.08014, 1.93032, 5.6674;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -14.8694,  3.37569,  2.29006, -1.41612,  2.96088,  11.4257 ;
  configurations.emplace_back(seedConfiguration);

  seedConfiguration << -2.13142, 3.31094, 2.04218, -2.27228, 3.94973,  3.93241;
  configurations.emplace_back(seedConfiguration);
  return configurations;
};

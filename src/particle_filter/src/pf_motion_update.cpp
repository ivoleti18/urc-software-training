#include "particle_filter/particle_filter.hpp"
#include <cmath>

namespace particle_filter
{



void ParticleFilter::applyMotionUpdate(
  double dx_body, double dy_body, double dyaw,
  double translation, double rotation)
{

  const double sigma_x = x_noise_ * translation;
  const double sigma_y = y_noise_ * translation;
  const double sigma_theta = theta_noise_ * rotation;

  for (auto & particle : particles_) {

    for (auto & particle : particles_) {
      // Sample noisy motion in body frame
      const double noisy_dx = dx_body + gaussianNoise(sigma_x);
      const double noisy_dy = dy_body + gaussianNoise(sigma_y);
      const double noisy_dyaw = dyaw + gaussianNoise(sigma_theta);

      const double c = std::cos(particle.theta);
      const double s = std::sin(particle.theta);

      const double world_dx = c * noisy_dx - s * noisy_dy;
      const double world_dy = s * noisy_dx + c * noisy_dy;

      particle.x += world_dx;
      particle.y += world_dy;
      particle.theta += wrapAngle(noisy_dyaw + particle.theta);
    }
  }
}



}  // namespace particle_filter

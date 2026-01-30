#include "localization/ParticleFilter.h"
#include "localization/Util.h"

#include "tf/tf.h"

using namespace std;

ParticleFilter::ParticleFilter(int numberOfParticles) {
	this->numberOfParticles = numberOfParticles;

	// initialize particles
	for (int i = 0; i < numberOfParticles; i++) {
		this->particleSet.push_back(new Particle());
	}

	// this variable holds the estimated robot pose
	this->bestHypothesis = new Particle();

	// at each correction step of the filter only the laserSkip-th beam of a scan should be integrated
	this->laserSkip = 5;

	// distance map used for computing the likelihood field
	this->distMap = NULL;
}

ParticleFilter::~ParticleFilter() {
	// delete particles
	for (int i = 0; i < numberOfParticles; i++) {
		Particle* p = this->particleSet[i];
		delete p;
	}

	this->particleSet.clear();

	if (this->likelihoodField)
		delete[] this->likelihoodField;

	delete this->bestHypothesis;

	if (this->distMap)
		delete[] this->distMap;
}

int ParticleFilter::getNumberOfParticles() {
	return this->numberOfParticles;
}

std::vector<Particle*>* ParticleFilter::getParticleSet() {
	return &(this->particleSet);
}

void ParticleFilter::initParticlesUniform() {
    //get map properties
    int mapWidth, mapHeight;
    double mapResolution;
    this->getLikelihoodField(mapWidth, mapHeight,mapResolution);

	// TODO: here comes your code
	// loop through each particle and assign a random x,y,theta
	for(int i = 0; i < this->numberOfParticles; i++) {
		double x = Util::uniformRandom(0, mapWidth) * mapResolution;
		double y = Util::uniformRandom(0, mapHeight) * mapResolution;
		double theta = Util::uniformRandom(0, 2 * M_PI);

		this->particleSet[i]->x = x;
		this->particleSet[i]->y = y;
		this->particleSet[i]->theta = theta;
	}
}

void ParticleFilter::initParticlesGaussian(double mean_x, double mean_y,
		double mean_theta, double std_xx, double std_yy, double std_tt) {
	// TODO: here comes your code
	// get the map properties
	int mapWidth, mapHeight;
	double mapResolution;
	this->getLikelihoodField(mapWidth, mapHeight, mapResolution);

	// loop through each particle and assign a random x,y,theta based on gaussian distribution
	for(int i = 0; i < this->numberOfParticles; i++) {
		// X coordinate
		double x_noisy = Util::gaussianRandom(mean_x, std_xx);
		double x = std::max(0.0, std::min(x_noisy, mapResolution * mapWidth));
		// Y coordinate
		double y_noisy = Util::gaussianRandom(mean_y, std_yy);
		double y = std::max(0.0, std::min(y_noisy, mapResolution * mapHeight));
		// Theta coordinate
		double theta_noisy = Util::gaussianRandom(mean_theta, std_tt);
		double theta = Util::normalizeTheta(theta_noisy);

		// assign to particle
		this->particleSet[i]->x = x;
		this->particleSet[i]->y = y;
		this->particleSet[i]->theta = theta;
	}
}

/**
 *  Initializes the likelihood field as our sensor model.
 */
void ParticleFilter::setMeasurementModelLikelihoodField(
		const nav_msgs::OccupancyGrid& map, double zRand, double sigmaHit) {
	ROS_INFO("Creating likelihood field for laser range finder...");

	// zRand is the probability of random measurements, thus (1 - zRand) is the weight for the Gaussian measurement

	// create the likelihood field - with the same discretization as the occupancy grid map
	this->likelihoodField = new double[map.info.height * map.info.width];
	this->likelihoodFieldWidth = map.info.width;
	this->likelihoodFieldHeight = map.info.height;
	this->likelihoodFieldResolution = map.info.resolution;

    // calculates the distance map and stores it in member variable 'distMap'
	// for every map position it contains the distance to the nearest occupied cell.
	calculateDistanceMap(map);

    // Here you have to create your likelihood field
	// HINT0: sigmaHit is given in meters. You have to take into account the resolution of the likelihood field to apply it.
	// HINT1: You will need the distance map computed 3 lines above
	// HINT2: You can visualize it in the map_view when clicking on "show likelihood field" and "publish all".
	// HINT3: Storing probabilities in each cell between 0.0 and 1.0 might lead to round-off errors, therefore it is
	// good practice to convert the probabilities into log-space, i.e. storing log(p(x,y)) in each cell. As a further
	// advantage you can simply add the log-values in your sensor model, when you weigh each particle according the
	// scan, instead of multiplying the probabilities, because: log(a*b) = log(a)+log(b).

	// TODO: here comes your code
	// Iterate over every cell in the map
	// convert sigmaHit from meters to cells
	double std_dev_cells = sigmaHit / this->likelihoodFieldResolution; // distance to nearest occupied cell in meters to how many cells
	// disMap is 1D array, index = x + y * width
	for(int i = 0; i < this->likelihoodFieldWidth * this->likelihoodFieldHeight; i++) {
		// dist to nearest occupied cell in cells
		double dist = this->distMap[i];

		// compute likelihood value mean how likely this cell hits an obstacle
		// P(z) = (1-zRand) * pHit + zRand , zRand is known, its random
		// pHit = zHit * Gaussian(dist, 0, std_dev_cells)
		// for pHit, zHit is weighting factor, in Gaussian, mean is 0, std deviation is std_dev_cells
		// in the map, for a specific pixel, the smaller the dist, the higher the likelihood this pixel hits an obstacle
		
		double zHit = 1.0 - zRand;
		double pHit = zHit * Util::gaussian(dist, std_dev_cells, 0.0); // x, std, mean
		double pz = pHit + zRand;
		// store log likelihood in the likelihood field
		this->likelihoodField[i] = std::log(pz);
	}	
	
	ROS_INFO("...DONE creating likelihood field!");
}

void ParticleFilter::calculateDistanceMap(const nav_msgs::OccupancyGrid& map) {
	// calculate distance map = distance to nearest occupied cell
	distMap = new double[likelihoodFieldWidth * likelihoodFieldHeight];
	int occupiedCellProbability = 90;
	// initialize with max distances
	for (int x = 0; x < likelihoodFieldWidth; x++) {
		for (int y = 0; y < likelihoodFieldHeight; y++) {
			distMap[x + y * likelihoodFieldWidth] = 32000.0;
		}
	}
	// set occupied cells next to unoccupied space to zero
	for (int x = 0; x < map.info.width; x++) {
		for (int y = 0; y < map.info.height; y++) {
			if (map.data[x + y * map.info.width] >= occupiedCellProbability) {
				bool border = false;
				for (int i = -1; i <= 1; i++) {
					for (int j = -1; j <= 1; j++) {
						if (!border && x + i >= 0 && y + j >= 0 && x + i
								< likelihoodFieldWidth && y + j
								< likelihoodFieldHeight && (i != 0 || j != 0)) {
							if (map.data[x + i + (y + j) * likelihoodFieldWidth]
									< occupiedCellProbability && map.data[x + i
									+ (y + j) * likelihoodFieldWidth] >= 0)
								border = true;
						}
						if (border)
							distMap[x + i + (y + j) * likelihoodFieldWidth]
									= 0.0;
					}
				}
			}
		}
	}
	// first pass -> SOUTHEAST
	for (int x = 0; x < likelihoodFieldWidth; x++)
		for (int y = 0; y < likelihoodFieldHeight; y++)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i
							< likelihoodFieldWidth && y + j
							< likelihoodFieldHeight && (i != 0 || j != 0)) {
						double v = distMap[x + i + (y + j)
								* likelihoodFieldWidth] + ((i * j != 0) ? 1.414
								: 1);
						if (v < distMap[x + y * likelihoodFieldWidth]) {
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}

	// second pass -> NORTHWEST
	for (int x = likelihoodFieldWidth - 1; x >= 0; x--)
		for (int y = likelihoodFieldHeight - 1; y >= 0; y--)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i
							< likelihoodFieldWidth && y + j
							< likelihoodFieldHeight && (i != 0 || j != 0)) {
						double v = distMap[x + i + (y + j)
								* likelihoodFieldWidth] + ((i * j != 0) ? 1.414
								: 1);
						if (v < distMap[x + y * likelihoodFieldWidth]) {
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}
}

double* ParticleFilter::getLikelihoodField(int& width, int& height,
		double& resolution) {
	width = this->likelihoodFieldWidth;
	height = this->likelihoodFieldHeight;
	resolution = this->likelihoodFieldResolution;

	return this->likelihoodField;
}

/**
 *  A generic measurement integration method that invokes some specific observation model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::measurementModel(
		const sensor_msgs::LaserScanConstPtr& laserScan) {
	likelihoodFieldRangeFinderModel(laserScan);
}

/**
 *  Method that implements the endpoint model for range finders.
 *  It uses a precomputed likelihood field to weigh the particles according to the scan and the map.
 */
void ParticleFilter::likelihoodFieldRangeFinderModel(
		const sensor_msgs::LaserScanConstPtr & laserScan) {
	
	// principle: at time z, laser detects obsctacle with distance l, then apply this l to all the particles
	// for each particle, after applying l, get cell in the map, if this cell has high likelihood(high chance of obstacl)
	// then we say this particle has high chance to be the true robot pose --> assign high weight to this particle
	// if the cell has low likelihood, then this particle has low chance to be the true robot pose --> assign low weight to this particle
	// TODO: here comes your code
	// loop through each particle
	for(int i = 0; i < this->numberOfParticles; i++) {
		// reset weight
		this->particleSet[i]->weight = 0.0;
		// loop through each laser beam, skip some beams according to laserSkip
		for(int j = 0; j < laserScan->ranges.size(); j += this->laserSkip) {
			double range = laserScan->ranges[j];
			// ignore max and min range readings
			if(range > laserScan->range_max || range < laserScan->range_min) {
				continue;
			}
			// calculate coordinate of end point in map frame
			double laserAngle = this->particleSet[i]->theta + laserScan->angle_min + j * laserScan->angle_increment;
			
			double hitX_meters = this->particleSet[i]->x + cos(laserAngle) * range;
			double hitY_meters = this->particleSet[i]->y + sin(laserAngle) * range;
			// convert to map cell coordinates
			int cellX = int(hitX_meters / this->likelihoodFieldResolution);
			int cellY = int(hitY_meters / this->likelihoodFieldResolution);

			// update particle weight by looking up cellX, cellY in likelihood field
			// check if outside map boundaries
			if(cellX >= 0 && cellX < this->likelihoodFieldWidth && cellY >= 0 && cellY < this->likelihoodFieldHeight) {
				// index in 1D array
				int index = cellX + cellY * this->likelihoodFieldWidth;
				// in practical, the total probability should be multiplied, but since we are in log space, we add them up
				// P(z1, z2, ..., zn) = P(z1) * P(z2) * ... * P(zn)
				// log P(z1, z2, ..., zn) = log ( P(z1) * P(z2) * ... * P(zn) ) = log P(z1) + log P(z2) + ... + log P(zn)
				this->particleSet[i]->weight += this->likelihoodField[index];
			} else {
				// if outside map, assign a very low likelihood
				this->particleSet[i]->weight += -100.0;
			}
		}
		// convert log(P(z1, z2, ..., zn)) back to P(z1, z2, ..., zn), and update as real weight
		// exp_log(P) = P
		this->particleSet[i]->weight = std::exp(this->particleSet[i]->weight);
	}


}

void ParticleFilter::setMotionModelOdometry(double alpha1, double alpha2,
		double alpha3, double alpha4) {
	this->odomAlpha1 = alpha1;
	this->odomAlpha2 = alpha2;
	this->odomAlpha3 = alpha3;
	this->odomAlpha4 = alpha4;

}

/**
 *  A generic motion integration method that invokes some specific motion model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::sampleMotionModel(double oldX, double oldY,
		double oldTheta, double newX, double newY, double newTheta) {
	sampleMotionModelOdometry(oldX, oldY, oldTheta, newX, newY, newTheta);
}

/**
 *  Method that implements the odometry-based motion model.
 */
void ParticleFilter::sampleMotionModelOdometry(double oldX, double oldY,
		double oldTheta, double newX, double newY, double newTheta) {
	// TODO: here comes your code
	// for each particle, compute its new position based on the encoder readings, also add noise

	//  get map properties
	int mapWidth, mapHeight;
	double mapResolution;
	this->getLikelihoodField(mapWidth, mapHeight, mapResolution);

	// compute motion without noise
	double delta_trans = std::sqrt((newX - oldX) * (newX - oldX) + (newY - oldY) * (newY - oldY));
	double delta_rot1 = std::atan2(newY - oldY, newX - oldX) - oldTheta;
	double delta_rot2 = newTheta - oldTheta - delta_rot1;
	// normalize angles
	delta_rot1 = Util::normalizeTheta(delta_rot1);
	delta_rot2 = Util::normalizeTheta(delta_rot2);

	// loop through each particle
	for(int i =0; i < this->numberOfParticles; i++) {
		// compute motion corrupted with noise
		// compute variances for each motion component
		double var_trans = this->odomAlpha3 * std::abs(delta_trans) + this->odomAlpha4 * (std::abs(delta_rot1) + std::abs(delta_rot2));
		double var_rot1 = this->odomAlpha1 * std::abs(delta_rot1) + this->odomAlpha2 * std::abs(delta_trans);
		double var_rot2 = this->odomAlpha1 * std::abs(delta_rot2) + this->odomAlpha2 * std::abs(delta_trans);

		// compute noisy motion components
		double noisy_delta_trans = Util::gaussianRandom(delta_trans, var_trans);
		double noisy_delta_rot1 = Util::gaussianRandom(delta_rot1, var_rot1);
		double noisy_delta_rot2 = Util::gaussianRandom(delta_rot2, var_rot2);

		// update particle position
		this->particleSet[i]->x += noisy_delta_trans * std::cos(this->particleSet[i]->theta + noisy_delta_rot1);
		this->particleSet[i]->y += noisy_delta_trans * std::sin(this->particleSet[i]->theta + noisy_delta_rot1);
		this->particleSet[i]->theta += noisy_delta_rot1 + noisy_delta_rot2;	

		// limit position to map boundaries and normalize theta (-pi to pi)
		this->particleSet[i]->x = std::max(0.0, std::min(this->particleSet[i]->x, mapResolution * mapWidth));
		this->particleSet[i]->y = std::max(0.0, std::min(this->particleSet[i]->y, mapResolution * mapHeight));
		this->particleSet[i]->theta = Util::normalizeTheta(this->particleSet[i]->theta);
	}
}

/**	
 *  The stochastic importance resampling.
 */
void ParticleFilter::resample() {
	// TODO: here comes your code

	// find the particle with the highest weight
	double maxWeight = -1.0;
	int bestIndex = 0;
	double weight_sum = 0.0;
	for(int i = 0; i < this->numberOfParticles; i++) {
		weight_sum += this->particleSet[i]->weight;
		if(this->particleSet[i]->weight > maxWeight) {
			maxWeight = this->particleSet[i]->weight;
			bestIndex = i;
		}
	}
	// update best hypothesis
	this->bestHypothesis->x = this->particleSet[bestIndex]->x;
	this->bestHypothesis->y = this->particleSet[bestIndex]->y;
	this->bestHypothesis->theta = this->particleSet[bestIndex]->theta;
	this->bestHypothesis->weight = this->particleSet[bestIndex]->weight;

	// stockastic universal resampling
	std::vector<Particle*> new_particle_set;
	
	double step = weight_sum / this->numberOfParticles;

	// we should only sample in one step range, so that all samples are evenly distributed in the whole weight range
	// the range of u and c is [0, weight_sum]
	double inner_position = (rand() / (double)RAND_MAX) * step; // rand() / RAND_MAX gives [0, 1), so u is in [0, step)
	double outer_position = this->particleSet[0]->weight;
	int i = 0;

	// inner_position is the inner arrows which are equally spaced
	// outer_position is the cumulative weight, outer pockets with different sizes


	for(int j = 0; j < this->numberOfParticles; j++) {
		// if the particle's weight is large, inner_position will be smaller than outer_position for multiple iterations
		// so i will not be updated, and we will sample multiple new particles from this old particle
		while(inner_position > outer_position){
			i++;
			if(i >= this->numberOfParticles) {
				i = this->numberOfParticles - 1;
				break;
			}

			outer_position += this->particleSet[i]->weight;
		}
		Particle* p = new Particle();
		p->x = this->particleSet[i]->x;
		p->y = this->particleSet[i]->y;
		p->theta = this->particleSet[i]->theta;
		p->weight = 1.0 / this->numberOfParticles; // reset weight
		new_particle_set.push_back(p);
		inner_position += step;
	}

	// delete old particles, avoid memory leak
	for(int k = 0; k < this->numberOfParticles; k++) {
		delete this->particleSet[k];
	}

	// replace old particle set with new one
	this->particleSet = new_particle_set;

}

Particle* ParticleFilter::getBestHypothesis() {
	return this->bestHypothesis;
}

// added for convenience
int ParticleFilter::computeMapIndex(int width, int height, int x,
		int y) {
	return x + y * width;
}


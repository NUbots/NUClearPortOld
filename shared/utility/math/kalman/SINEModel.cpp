#include <math.h> //needed for normalisation function
#include <assert.h>
#include "SINEModel.h" //includes armadillo


//#include "Tools/Math/General.h" //replace with armadillo library
//#include <assert.h>

//---------------------originally included in Math/general.h ::these functions can (and should) be moved to utilites if used by other files
//const double PI = 4.0*atan(1.0);          //this already exists as M_PI in the <cmath> aka <math.h> library, redundant calculation
inline double normaliseAngle(double theta){ //a helper function (originally in Math/general.h - required, can be ported to utilities)
    return atan2(sin(theta), cos(theta));
}
template <class T> inline int sign(T x){
  if(x < 0.0) return -1;
  else if(x > 0.0) return 1;
  else return 0;
}
//---------------------

SINEModel::SINEModel() {
}

SINEModel::SINEModel(const SINEModel& source) {
    *this = source;
}

void SINEModel::limitState(arma::mat &state) {
    const float pi_2 = 0.5 * M_PI; 
    if(fabs(state(kstates_body_angle_x, 0)) > pi_2 and fabs(state(kstates_body_angle_y, 0)) > pi_2) { // This part checks for the event where a large roll and large pitch puts the robot back upright
        state(kstates_body_angle_x, 0) = state(kstates_body_angle_x, 0) - sign(state(kstates_body_angle_x, 0)) * M_PI;
        state(kstates_body_angle_y, 0) = state(kstates_body_angle_y, 0) - sign(state(kstates_body_angle_y, 0)) * M_PI;
    }

    // Regular unwrapping.
    state(kstates_body_angle_x, 0) = normaliseAngle(state(kstates_body_angle_x, 0));
    state(kstates_body_angle_y, 0) = normaliseAngle(state(kstates_body_angle_y, 0));
    return;
}

arma::mat SINEModel::kinematicMeasurementEquation(const arma::mat& state, const arma::mat& measurementArgs) {
    // measurementArgs contain no data.
    // Measurement is returned in angle [theta_x, theta_y]^T.

    arma::mat result(2,1);
    result(0, 0) = state(kstates_body_angle_x, 0);
    result(1, 0) = state(kstates_body_angle_y, 0);
    return result;
}

arma::mat SINEModel::accelerometerMeasurementEquation(const arma::mat& state, const arma::mat& measurementArgs) {
    // measurementArgs contain no data.
    // Measurement is returned as accelerations [x, y, z]^T.
    // Gravity vector - Starts as pointing down, z =s 980.7 cm/s^2 as
    
    arma::mat g_vec(3, 1);
    g_vec(0, 0) = 0.f;
    g_vec(1, 0) = 0.f;
    g_vec(2, 0) = -980.7f;

    const double body_roll  = state(kstates_body_angle_x, 0); //state ~ input parameter
    const double body_pitch = state(kstates_body_angle_y, 0);
    float sinA, cosA;

    arma::mat body_roll_rot = arma::mat(3,3);
    body_roll_rot.eye(); //body_roll_rot should be initialised as an identity matrix
    sinA = sin(body_roll);
    cosA = cos(body_roll);
    body_roll_rot(1, 1) = cosA;
    body_roll_rot(1, 2) = sinA;
    body_roll_rot(2, 1) = -sinA;
    body_roll_rot(2, 2) = cosA;

    arma::mat body_pitch_rot = arma::mat(3,3);
    body_pitch_rot.eye(); //generate identity matrix
    sinA = sin(body_pitch);
    cosA = cos(body_pitch);
    body_pitch_rot(0, 0) = cosA;
    body_pitch_rot(0, 2) = -sinA;
    body_pitch_rot(2, 0) = sinA;
    body_pitch_rot(2, 2) = cosA;

    arma::mat result = body_roll_rot * body_pitch_rot * g_vec; //this line armadillo hates ..........................................!
    return result;
}

std::ostream& SINEModel::writeStreamBinary (std::ostream& output) const { // @brief Outputs a binary representation of the UKF object to a stream. @param output The output stream. @return The output stream.
    return output;
}

// @brief Reads in a UKF object from the input stream. @param input The input stream. @return The input stream.
std::istream& SINEModel::readStreamBinary (std::istream& input) {
    return input;
}

//------------------------------------------------------------------------------
arma::mat SINEModel::processEquation(const arma::mat& state, double deltaT, const arma::mat& measurement) { // @brief The process equation is used to update the systems state using the process euquations of the system. @param sigma_point The sigma point representing a system state. @param deltaT The amount of time that has passed since the previous update, in seconds. @param measurement The reading from the rate gyroscope in rad/s used to update the orientation. @return The new estimated system state.
    //arma::mat result(state); // Start at original state.
    //result(kstates_body_angle_x, 0) += (measurement(0, 0) - state(kstates_gyro_offset_x, 0)) * deltaT; // Add measurement + offset.
    //result(kstates_body_angle_y, 0) += (measurement(1, 0) - state(kstates_gyro_offset_y, 0)) * deltaT;
    //limitState(result);
    arma::mat result(state); 
    
}

arma::mat SINEModel::measurementEquation(const arma::mat& state, const arma::mat& measurementArgs, unsigned int type) { // @brief The measurement equation is used to calculate the expected measurement given a system state. @param sigma_point The sigma point representing a system state. @param measurementArgs Additional arguments used to calculate the measurement. In this implementation it remains empty. @return The expected measurement for the given states. Either accelerometer measurement, or kinematic measurement.
    arma::mat result;
    switch(type) {
        case kmeasurement_accelerometer:
            result = accelerometerMeasurementEquation(state, measurementArgs);
            break;
        case kmeasurement_kinematic:
            result = kinematicMeasurementEquation(state, measurementArgs);
            break;
    };
    return result;
}

arma::mat SINEModel::measurementDistance(const arma::mat& measurement1, const arma::mat& measurement2, unsigned int type) {
    arma::mat result;
    switch(type)
    {
        case kmeasurement_accelerometer:
            result = measurement1 - measurement2;
            break;
        case kmeasurement_kinematic:
            result = measurement1 - measurement2;
            result(0, 0) = normaliseAngle(result(0, 0));
            result(1, 0) = normaliseAngle(result(1, 0));
            break;
    };
    return result;
}
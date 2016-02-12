#include "Kinematics.hpp"

using std::cout;
using std::endl;


namespace shared {

Kinematics::Kinematics():
    links_(),
	joint_origins_(),
	joint_axis_(){
}

void Kinematics::setJointOrigins(std::vector<std::vector<double>> &joint_origins) {
	for (auto &o: joint_origins) {
		joint_origins_.push_back(o);		
	}
}

void Kinematics::setJointAxis(std::vector<std::vector<int>> &axis) {
	cout << "joint axes: " << endl;
	for (auto &o: axis) {
		for (auto &k: o) {
			cout << k << ", ";
		}
		cout << endl;
		joint_axis_.push_back(o);		
	}
}

void Kinematics::setLinkDimensions(std::vector<std::vector<double>> &link_dimensions) {
	for (auto &k: link_dimensions) {
		links_.push_back(k);
	}
}

void Kinematics::getPositionOfLinkN(const std::vector<double> &joint_angles, const int &n, std::vector<double> &position) const {
    std::pair<fcl::Vec3f, fcl::Matrix3f> link_n_pose = getPoseOfLinkN(joint_angles, n);
    position.push_back(link_n_pose.first[0]);
    position.push_back(link_n_pose.first[1]);
    position.push_back(link_n_pose.first[2]);
}

/* Gets the end effector position for a given set of joint angles */    
void Kinematics::getEndEffectorPosition(const std::vector<double> &joint_angles, std::vector<double> &end_effector_position) const {
    int n = joint_angles.size();	
    std::pair<fcl::Vec3f, fcl::Matrix3f> ee_pose = getPoseOfLinkN(joint_angles, n);
    end_effector_position.push_back(ee_pose.first[0]);
    end_effector_position.push_back(ee_pose.first[1]);
    end_effector_position.push_back(ee_pose.first[2]);
}  

std::pair<fcl::Vec3f, fcl::Matrix3f> Kinematics::getPoseOfLinkN(const std::vector<double> &joint_angles, const int &n) const {    
   Eigen::MatrixXd res = Eigen::MatrixXd::Identity(4, 4);
   Eigen::MatrixXd init_trans(4, 4);
   init_trans << 1.0, 0.0, 0.0, joint_origins_[0][0], 
		         0.0, 1.0, 0.0, joint_origins_[0][1],
				 0.0, 0.0, 1.0, joint_origins_[0][2],
				 0.0, 0.0, 0.0, 1.0;
   std::vector<Eigen::MatrixXd> transformations;
   transformations.push_back(init_trans);   
   for (unsigned int i = 0; i < n; i++) {	   
	   Eigen::MatrixXd t2 = transform(joint_origins_[i + 1][0], 
			                          joint_origins_[i + 1][1], 
			                          joint_origins_[i + 1][2],
			                          joint_origins_[i + 1][3], 
			                          joint_origins_[i + 1][4], 
			                          joint_origins_[i + 1][5]);
	   
	   Eigen::MatrixXd t1 = transform(0.0, 
			                          0.0, 
			                          0.0, 
			                          joint_angles[i] * joint_axis_[i][0],
			                          joint_angles[i] * joint_axis_[i][1],
			                          joint_angles[i] * joint_axis_[i][2]);	   
	   transformations.push_back(t1 * t2);
   }
   
   transformations.push_back(transform(0.0, 
		                               0.0, 
		                               0.0, 
		                               joint_angles[n] * joint_axis_[n][0],
		                               joint_angles[n] * joint_axis_[n][1],
		                               joint_angles[n] * joint_axis_[n][2]));
   for (int i = 0; i < transformations.size(); i++) {	   
       res = res * transformations[i];
   }
   
   fcl::Vec3f r_vec = fcl::Vec3f(res(0, 3), res(1, 3), res(2, 3));
   fcl::Matrix3f r_matr = fcl::Matrix3f(res(0, 0), res(0, 1), res(0, 2), 
                                        res(1, 0), res(1, 1), res(1, 2), 
                                        res(2, 0), res(2, 1), res(2, 2));   
   auto p = std::make_pair(r_vec, r_matr);   
   return p;
}

std::pair<fcl::Vec3f, fcl::Matrix3f> Kinematics::getEndEffectorPose(const std::vector<double> &joint_angles) const {
	int n = joint_angles.size(); 
    return getPoseOfLinkN(joint_angles, n);
}

void Kinematics::getEEJacobian(const std::vector<double> &joint_angles, Eigen::MatrixXd &jacobian) {
	auto ee_pose = getEndEffectorPose(joint_angles);	
	Eigen::Vector3d o_e;
	o_e << ee_pose.first[0], ee_pose.first[1], ee_pose.first[2];	
	std::vector<std::pair<fcl::Vec3f, fcl::Matrix3f>> link_poses;
	for (size_t i = 0; i < joint_angles.size(); i++) {
		auto pose = getPoseOfLinkN(joint_angles, i);
		Eigen::VectorXd column_vector(6);
		Eigen::Vector3d o_i;
		o_i << pose.first[0], pose.first[1], pose.first[2];		
		Eigen::Vector3d z_i;
		z_i << pose.second(0, 2), pose.second(1, 2), pose.second(2, 2);		
		Eigen::Vector3d upper = z_i.cross(o_e - o_i);
		column_vector << upper, z_i;
		jacobian.col(i) = column_vector;
	}
}

Eigen::MatrixXd Kinematics::transform(double x, double y, double z, double roll, double pitch, double yaw) const{
	Eigen::MatrixXd trans(4, 4);
	trans << 1.0, 0.0, 0.0, x,
			 0.0, 1.0, 0.0, y,
			 0.0, 0.0, 1.0, z,
			 0.0, 0.0, 0.0, 1.0;
	
	Eigen::MatrixXd ro(4, 4);
	ro << 1.0, 0.0, 0.0, 0.0,
		 0.0, cos(roll), -sin(roll), 0.0,
		 0.0, sin(roll), cos(roll), 0.0,
		 0.0, 0.0, 0.0, 1.0;
	
	Eigen::MatrixXd pi(4, 4);
	pi << cos(pitch), 0.0, sin(pitch), 0.0,
		 0.0, 1.0, 0.0, 0.0,
		 -sin(pitch), 0.0, cos(pitch), 0.0,
		 0.0, 0.0, 0.0, 1.0;
	
	Eigen::MatrixXd ya(4, 4);
	ya << cos(yaw), -sin(yaw), 0.0, 0.0,
		  sin(yaw), cos(yaw), 0.0, 0.0,
		  0.0, 0.0, 1.0, 0.0,
		  0.0, 0.0, 0.0, 1.0;
	
	Eigen::MatrixXd res = ro * pi * ya * trans;
	return res;
}

Eigen::MatrixXd Kinematics::getTransformationMatr(double sigma_n, double d_n, double a_n, double alpha_n) const {
    Eigen::MatrixXd b(4,4);    
    b << cos(sigma_n), -sin(sigma_n) * cos(alpha_n), sin(sigma_n) * sin(alpha_n), a_n * cos(sigma_n),
         sin(sigma_n), cos(sigma_n) * cos(alpha_n), -cos(sigma_n) * sin(alpha_n), a_n * sin(sigma_n),
         0.0, sin(alpha_n), cos(alpha_n), d_n,
         0.0, 0.0, 0.0, 1.0;
    return b;
}
 
}
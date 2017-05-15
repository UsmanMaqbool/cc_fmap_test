#include "HelperFcts.h"

using namespace std;
using namespace cv;


HelperFcts::HelperFcts(){}
HelperFcts::~HelperFcts(){}

void HelperFcts::calc3DPt(Point2f pt2D, float fx, float fy, float cx, float cy, Mat idepth, Eigen::Vector3f & pt3D) {
	pt3D << pt2D.x*1/fx + (-cx/fx), pt2D.y*1/fy + (-cy/fy), 1.0;
	float depth = 1/idepth.at<float>(pt2D.x,pt2D.y);

	pt3D = pt3D*depth;
}

void HelperFcts::displayImage(string windowName, Mat img) {
    imshow(windowName, img);
    waitKey(10);
}

void HelperFcts::displayImageKPts(string windowName, Mat img, vector<KeyPoint> kPts) {
	Mat imgModified;
	drawKeypoints(img, kPts, imgModified);
    imshow(windowName, imgModified);
    waitKey(10);
}

void HelperFcts::displayMatches(string windowName, Mat img1, Mat img2, vector<KeyPoint> kPts1, vector<KeyPoint> kPts2, vector<DMatch> matches1to2, bool boolSaveImg) {
	Mat imgModified;
	drawMatches(img1, kPts1, img2, kPts2, matches1to2, imgModified);
    imshow(windowName, imgModified);
    waitKey(10);
    if(boolSaveImg == true) {
    	HelperFcts::saveImage(imgModified,"testImgs/displayMatches.jpg");
    }
}

void HelperFcts::displaySaveImage(string windowName, Mat img, string nameInclingDir) {
    imshow(windowName, img);
    waitKey(10);
    imwrite(nameInclingDir, img);
}

void HelperFcts::eigenMatrix4fToROSTrafoMsg(Eigen::Matrix4f trafo, tf::Transform & trafoROSMsg) {
	tf::Vector3 origin( trafo(0,3), trafo(1,3), trafo(2,3) );

	Eigen::Matrix3f rot = trafo.block(0,0,3,3);
	Eigen::Quaternion<float> qEigen(rot);
	tf::Quaternion q( qEigen.x(), qEigen.y(), qEigen.z() ,qEigen.w() );
	q.normalize();

	trafoROSMsg.setOrigin(origin);
	trafoROSMsg.setRotation(q);
}

void HelperFcts::eigenQuatTranslPoseToROSPose(Eigen::Quaternion<float> quat, Eigen::Translation<float,3> transl, geometry_msgs::PoseStamped & pose) {
	pose.pose.position.x = transl.x();
	pose.pose.position.y = transl.y();
	pose.pose.position.z = transl.z();

	quat.normalize();
	pose.pose.orientation.w = quat.w();
	pose.pose.orientation.x = quat.x();
	pose.pose.orientation.y = quat.y();
	pose.pose.orientation.z = quat.z();
}

void HelperFcts::invQuatTranslScale(Eigen::Quaternion<float> quat, Eigen::Quaternion<float> & quatInv, Eigen::Translation<float,3> transl, Eigen::Translation<float,3> & translInv, float sc, float & scInv) {
	quatInv = quat.inverse();

	Eigen::Vector3f translTmp1,translTmp2;
	translTmp1[0] = transl.x();
	translTmp1[1] = transl.y();
	translTmp1[2] = transl.z();
	translTmp2 = -quatInv.toRotationMatrix()*translTmp1;

	translInv.x() = translTmp2[0];
	translInv.y() = translTmp2[1];
	translInv.z() = translTmp2[2];

	scInv = 1/sc;
}

void HelperFcts::invTrafo(Eigen::Matrix4f trafo, Eigen::Matrix4f & trafoInv) {
	trafoInv.block(0,0,3,3) = trafo.block(0,0,3,3).transpose();
	trafoInv.block(0,3,3,1) = -trafo.block(0,0,3,3).transpose()*trafo.block(0,3,3,1);
}

void HelperFcts::poseStampedROSToMatrix4f(geometry_msgs::PoseStamped poseROS, Eigen::Matrix4f & poseMat) {
	poseMat.setIdentity(4,4);

	Eigen::Quaternion<float> quat;
	quat.w() = poseROS.pose.orientation.w;
	quat.x() = poseROS.pose.orientation.x;
	quat.y() = poseROS.pose.orientation.y;
	quat.z() = poseROS.pose.orientation.z;


	poseMat.block(0,0,3,3) = quat.toRotationMatrix();
	poseMat.block(0,3,3,1) << poseROS.pose.position.x, poseROS.pose.position.y, poseROS.pose.position.z;
}

void HelperFcts::poseROSToTrafoROSMsg(geometry_msgs::Pose trafo,tf::Transform & trafoROSMsg) {
	tf::Vector3 origin(trafo.position.x, trafo.position.y, trafo.position.z);
	tf::Quaternion q;
	q.setW(trafo.orientation.w);
	q.setX(trafo.orientation.x);
	q.setY(trafo.orientation.y);
	q.setZ(trafo.orientation.z);
	q.normalize();

	trafoROSMsg.setOrigin(origin);
	trafoROSMsg.setRotation(q);
}

void HelperFcts::saveImage(Mat img, string nameInclingDir) {
    imwrite(nameInclingDir, img);
}

void HelperFcts::saveMatrix(Mat matrix, string nameInclingDir) {
	cv::FileStorage fs(nameInclingDir, cv::FileStorage::WRITE);
	// Write to file!
	fs << "matchMatrix" << matrix;
	fs.release();
}

void HelperFcts::saveStringToFile(string content, string nameInclingDir) {
	cv::FileStorage fs(nameInclingDir, cv::FileStorage::WRITE);
	// Write to file!
	fs << "CONTENT" << content;
	fs.release();
}

void HelperFcts::trafoFloat7Matrix4f(float trafoFloat[7], Eigen::Matrix4f & trafoMat) {
	trafoMat.setIdentity(4,4);

	Eigen::Quaternion<float> quat;
	quat.x() = trafoFloat[0];
	quat.y() = trafoFloat[1];
	quat.z() = trafoFloat[2];
	quat.w() = trafoFloat[3];

	float sc = quat.norm();
	quat.normalize();

	trafoMat.block(0,0,3,3) = quat.toRotationMatrix();
	trafoMat.block(0,3,3,1) << trafoFloat[4], trafoFloat[5], trafoFloat[6];
}

void HelperFcts::trafoFloat7ToQuatTranslScale(float trafoFloat[7], Eigen::Quaternion<float> & quat, Eigen::Translation<float,3> & transl, float & sc) {
	quat.x() = trafoFloat[0];
	quat.y() = trafoFloat[1];
	quat.z() = trafoFloat[2];
	quat.w() = trafoFloat[3];

	sc = quat.norm();

	quat.normalize();

	transl.x() = trafoFloat[4];
	transl.y() = trafoFloat[5];
	transl.z() = trafoFloat[6];
}

void HelperFcts::transform3DPt(float simTrafo[7], Eigen::Vector3f pt3D, Eigen::Vector3f & pt3DTransformed) {
	Eigen::Matrix4f trafoTmp(Eigen::Matrix4f::Identity(4,4));
	Eigen::Vector4f pt3DTmp;

	HelperFcts::trafoFloat7Matrix4f(simTrafo,trafoTmp);

	pt3DTmp << pt3D(0), pt3D(1), pt3D(2), 1.0;
	pt3DTmp = trafoTmp*pt3DTmp;
	pt3DTmp = pt3DTmp/pt3DTmp(3);

	pt3DTransformed(0) = pt3DTmp(0);
	pt3DTransformed(1) = pt3DTmp(1);
	pt3DTransformed(2) = pt3DTmp(2);
}

void HelperFcts::transform3DPt(Eigen::Matrix4f simTrafo, Eigen::Vector3f pt3D, Eigen::Vector3f & pt3DTransformed) {
	Eigen::Vector4f pt3DTmp;

	pt3DTmp << pt3D(0), pt3D(1), pt3D(2), 1.0;
	pt3DTmp = simTrafo*pt3DTmp;
	pt3DTmp = pt3DTmp/pt3DTmp(3);

	pt3DTransformed(0) = pt3DTmp(0);
	pt3DTransformed(1) = pt3DTmp(1);
	pt3DTransformed(2) = pt3DTmp(2);
}


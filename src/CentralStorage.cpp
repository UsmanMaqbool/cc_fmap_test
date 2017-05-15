#include "CentralStorage.h"

using namespace std;
using namespace opengv;


// Constructor
CentralStorage::CentralStorage(int nbrRobotsInput, int nbrVisWordsInput) :
	nbrRobots(nbrRobotsInput),
	nbrVisWords(nbrVisWordsInput),
	testImgCtr(0),
	stopDataCollection(false)
{
//	// Init camera Matrix and distortion coefficients
//	//float camMatArr[3][3] = {{442.777026, 0, 291.591624},{0, 444.581889, 207.395871},{0, 0, 1}};
//	//float distCoeffsArr[5] = {-0.343887, 0.100088, -0.001316, -0.000163, 0};
//	//camMat = Mat(3,3,CV_32FC1,camMatArr);
//	//distCoeffs = Mat(5,1,CV_32FC1,distCoeffsArr);



	// Detector, Extractor, Matcher, BOW Img Descriptor, BOW KMeans trainer
	//detector = new DynamicAdaptedFeatureDetector(AdjusterAdapter::create("SURF"),150,250,4); //
	//detector = new DynamicAdaptedFeatureDetector(AdjusterAdapter::create("SIFT"),400,500,5); //
	//detector = new SiftFeatureDetector(0, 1, 1, 5, 0.9); // nfeatures, noctavelayers,contrastthreshold (higher->less features), edgethreshold (higher->more features) ,sigma
	detector = new SiftFeatureDetector(0, 4, 0.04, 10, 1.0); // nfeatures, noctavelayers,contrastthreshold (higher->less features), edgethreshold (higher->more features) ,sigma

//detector = new SiftFeatureDetector(0, 3, 0.15, 5, 0.9);0, 3, 0.15, 5, 0.9

	//extractor = new SurfDescriptorExtractor(500,4,2,false,true); //hessian threshold, noctave,noctavelayers,extended,upright
	extractor = new SiftDescriptorExtractor(); //hessian threshold, noctave,noctavelayers,extended,upright
	matcher = DescriptorMatcher::create("FlannBased");
	bide = 	new BOWImgDescriptorExtractor(extractor, matcher);
	// clustering (nbr clusters = nbr vis words)
	//define Term Criteria
	TermCriteria tc(CV_TERMCRIT_ITER, 100, 0.001);
	//retries number
	int retries = 1;
	//necessary flags
	int flags = KMEANS_PP_CENTERS;
	BOWTrainer = new BOWKMeansTrainer(nbrVisWords, tc, retries, flags);

	// Initializing Kalman Filter for scale estimation
	KalmanFilterScale::Vector x(1);
	x(1) = 1.0;
	static const float _P0[] = {10.0};
	KalmanFilterScale::Matrix P0(1, 1, _P0);
	filter.init(x, P0);

	// Initialize global Transforms with identity transform and initialize ptClsUpdated to true
	for (int i = 1; i <= nbrRobots; ++i) {
		pRobotsWorld[i] = Eigen::Matrix4f::Identity(4, 4);
		ptClsInGlobalWorldCOSUpdated[i] = false;
		robotScales[i] = 1.0;
	}

}

CentralStorage::~CentralStorage() {}


void CentralStorage::generateVocabulary() {
	this->BOWTrainer->add(this->trainDescriptors_allRobots);
	this->vocabulary = this->BOWTrainer->cluster(); // Returns the cluster centers (descriptors are clustered into nbr vis words)
}

void CentralStorage::computeTrainBOWDescriptors() {
	int imgCtr = 0;
	cv::Mat BOWDescriptors;

	std::vector<cv::Mat>::iterator itImg;
	for (itImg = this->trainImgs_allRobots.begin(); itImg != this->trainImgs_allRobots.end(); ++itImg) {
		this->bide->compute(*itImg, this->trainKPts_allRobots[imgCtr], BOWDescriptors);
		this->trainBOWDescriptors_allRobots.push_back(BOWDescriptors);
		imgCtr = imgCtr + 1;
	}
}

void CentralStorage::generateCLTree() {
	of2::ChowLiuTree treeBuilder;
	treeBuilder.add(this->trainBOWDescriptors_allRobots);
	this->clTree = treeBuilder.make();
}

void CentralStorage::searchForGoodMatch() {
	//FOR TMR
	// dont consider first several imsg
	// add consecutive if from other robot
	// no match if other robot has some there
	if (this->testImgCtr > 0) {

		ostringstream convStringFileName, convStringContent;
		string matchedImgsFileName, matchedImgsContent;
		int kFrameNbr = -10000;
		int rID = this->kFrames[this->testImgCtr].rID;
		vector<of2::IMatch>::const_iterator it;
		float maxLikeli = 0;
		float matchCurrent(0.0), matchLast(0.0), matchSecondLast(0.0);
		//int nbrImgs = (int) -0.5+(1/4+2*this->matches.size())^(1/2);
		// for last query Img (last keyframe) loop through all (test)imgs except the query img on its own (query img - query img : this->matches.end()-this->testImgCtr-1)
		for (it = this->matches.end() - this->testImgCtr; it != this->matches.end(); ++it) {

//			if(this->testImgCtr < 30) { // dont consider matches among the first 6 keyframes
//			if(this->testImgCtr < 8) { // dont consider matches among the first 6 keyframes
			if (this->testImgCtr < 45) { // dont consider matches among the first 6 keyframes
				continue;
			}


//			if(it->match < 0.75) {
			if (it->match < 0.75) {
//				cout << "		match probability too low" << endl;
				continue;
			}

			if (it->imgIdx != -1) { // comparing query Img to testing Img associated with kFrameNbr
				kFrameNbr = it->imgIdx;
			}
			else if (it->imgIdx == -1) { // comparing query Img to itself
//				cout << "		compared to its own2" << endl;
				kFrameNbr = this->kFrames.size() - 1;
				continue; // Do not test to itself
			}

			if (this->kFrames[this->testImgCtr].rID == this->kFrames[kFrameNbr].rID) { // No matches of same robot
//				cout << "		same robot ID" << endl;
				continue;
			}

			// name of file determines queryImg specifications
			convStringFileName << "matchingImgs/robot" << this->kFrames[this->testImgCtr].rID << "MatchedImgsQueryImg" << this->kFrames[this->testImgCtr].fID << ".yml";
			matchedImgsFileName = convStringFileName.str();

			convStringContent << "robot: " << this->kFrames[kFrameNbr].rID << "   MatchedImg: " << this->kFrames[kFrameNbr].fID << "   MatchProbab: " << it->match << "     ";
			matchedImgsContent = convStringContent.str();

			this->matchedKeyFrames.first = this->testImgCtr; // query Img
			this->matchedKeyFrames.second = kFrameNbr; // matched test Img
			this->matchedKeyFramesMap.insert(std::pair<int, std::pair <int, int> >(this->testImgCtr, this->matchedKeyFrames)); // store with key "nbr of query image (= this->testImgCtr)"
			if (!matchedImgsContent.empty()) {
				cout << "		GOOD MATCH" << endl;
				HelperFcts::saveStringToFile(matchedImgsContent, matchedImgsFileName);
				this->boolMatch = true;

				// set bool to update map
				typedef std::map<int, bool> ptClUpdatedMap;
				for (ptClUpdatedMap::iterator it = this->ptClsInGlobalWorldCOSUpdated.begin(); it != this->ptClsInGlobalWorldCOSUpdated.end(); ++it) {
					it->second = false;
				}
			}
		} // END for loop through matches

	} // END if this->testImgCtr > 0
}

void CentralStorage::findTrafoInitialGuess() {

	cout << "		find IG" << endl;
	int queryImgNbr = this->testImgCtr;
	int testImgNbr = matchedKeyFramesMap[queryImgNbr].second;

	keyFrame queryKF, testKF;

	queryKF = this->kFrames[queryImgNbr];
	testKF = this->kFrames[testImgNbr];
	int queryRID = queryKF.rID;
	int testRID = testKF.rID;
	// Extract descriptors from imgs
	Mat queryImgDescr, testImgDescr;
	this->extractor->compute(queryKF.img, queryKF.KPts, queryImgDescr);
	this->extractor->compute(testKF.img, testKF.KPts, testImgDescr);

	// match descriptors
	std::vector< DMatch > resultingMatches;
	this->matcher->match(queryImgDescr, testImgDescr, resultingMatches);

	// Filter out bad matches
	double max_dist = 0; double min_dist = 100;
	//-- Quick calculation of max and min distances between keypoints
	for ( int i = 0; i < queryImgDescr.rows; i++ ) {
		double dist = resultingMatches[i].distance;
		if ( dist < min_dist ) min_dist = dist;
		if ( dist > max_dist ) max_dist = dist;
	}
	std::vector< DMatch > good_matches;
	for ( int i = 0; i < queryImgDescr.rows; i++ ) {
		if ( resultingMatches[i].distance <= max(3 * min_dist, 10.0) ) {
			good_matches.push_back( resultingMatches[i]);
		}
	}


	HelperFcts::displayMatches("testmatches", queryKF.img, testKF.img, queryKF.KPts, testKF.KPts, good_matches, true);

	//derive correspondences based on random point-cloud
	bearingVector_t bearingVectorQuery, bearingVectorTest;
	bearingVectors_t bearingVectorsQuery;
	bearingVectors_t bearingVectorsTest;

	vector<Point2f> matchedPtsQuery, matchedPtsTest;
	for (int i = 0; i < good_matches.size(); ++i)
	{
		Point2f matchedPtQuery = queryKF.KPts[good_matches[i].queryIdx].pt;
		Point2f matchedPtTest = testKF.KPts[good_matches[i].trainIdx].pt;

		matchedPtsQuery.push_back(matchedPtQuery);
		matchedPtsTest.push_back(matchedPtTest);

		bearingVectorQuery[0] = matchedPtQuery.x / sqrt(matchedPtQuery.x * matchedPtQuery.x + matchedPtQuery.y * matchedPtQuery.y + 1.0);
		bearingVectorQuery[1] = matchedPtQuery.y / sqrt(matchedPtQuery.x * matchedPtQuery.x + matchedPtQuery.y * matchedPtQuery.y + 1.0);
		bearingVectorQuery[2] = 1.0;//sqrt(1.0 - matchedPtQuery.x*matchedPtQuery.x - matchedPtQuery.y*matchedPtQuery.y);
		bearingVectorQuery.normalize();

		bearingVectorTest[0] = matchedPtTest.x / sqrt(matchedPtTest.x * matchedPtTest.x + matchedPtTest.y * matchedPtTest.y + 1.0);
		bearingVectorTest[1] = matchedPtTest.y / sqrt(matchedPtTest.x * matchedPtTest.x + matchedPtTest.y * matchedPtTest.y + 1.0);
		//sqrt(1.0 - matchedPtTest.x*matchedPtTest.x - matchedPtTest.y*matchedPtTest.y)
		bearingVectorTest[2] = 1.0;//sqrt(1.0 - matchedPtTest.x*matchedPtTest.x - matchedPtTest.y*matchedPtTest.y);
		bearingVectorTest.normalize();

		bearingVectorsQuery.push_back(bearingVectorQuery);
		bearingVectorsTest.push_back(bearingVectorTest);

	}

	relative_pose::CentralRelativeAdapter adapter(bearingVectorsQuery, bearingVectorsTest);
	sac::Ransac<sac_problems::relative_pose::CentralRelativePoseSacProblem> ransac;
	std::shared_ptr<sac_problems::relative_pose::CentralRelativePoseSacProblem> relposeproblem_ptr(new sac_problems::relative_pose::CentralRelativePoseSacProblem(adapter, sac_problems::relative_pose::CentralRelativePoseSacProblem::NISTER));
	ransac.sac_model_ = relposeproblem_ptr;
	ransac.threshold_ = 2.0 * (1.0 - cos(atan(sqrt(2.0) * 0.5 / 800.0)));
	ransac.max_iterations_ = 50;

	ransac.computeModel();
	transformation_t pCamTestCamQueryOpenGV = ransac.model_coefficients_;


	Eigen::Matrix4f iG(Eigen::Matrix4f::Identity(4, 4));
	Eigen::Matrix4d iGdouble(Eigen::Matrix4d::Identity(4, 4));
	iGdouble.block(0, 0, 3, 4) = pCamTestCamQueryOpenGV;
	iG = iGdouble.cast<float>();

	this->iGMap[queryImgNbr] = std::make_pair(iG, testImgNbr);

}

void CentralStorage::findTrafo(Eigen::Matrix4f iG, Eigen::Matrix4f & finalTrafo) {

	int queryImgCtrMatch = this->testImgCtr;
	int testImgCtrMatch = this->matchedKeyFramesMap[queryImgCtrMatch].second;

	pcl::PointCloud<pcl::PointXYZ>::Ptr ptClToPtrSource(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptClToPtrTarget(new pcl::PointCloud<pcl::PointXYZ>());
	*ptClToPtrSource = this->ptCls[queryImgCtrMatch].second;
	*ptClToPtrTarget = this->ptCls[testImgCtrMatch].second;

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(ptClToPtrSource);
	icp.setInputTarget(ptClToPtrTarget);

	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final, iG);

	finalTrafo = icp.getFinalTransformation();
}

void CentralStorage::estimateScale() {
	cout << "	estimate scale " << endl;

	// if query = robot 2
	// iGTrafo = pC1C2 = tC2C1
	int queryImgNbr = this->testImgCtr;
	int testImgNbr = matchedKeyFramesMap[queryImgNbr].second;
	Eigen::Matrix4f iGTrafo = this->iGMap[queryImgNbr].first;
	keyFrame queryKF, testKF;


	queryKF = this->kFrames[queryImgNbr];
	testKF = this->kFrames[testImgNbr];
	int queryRID = queryKF.rID;
	int testRID = testKF.rID;

	// Extract descriptors from imgs
	Mat queryImgDescr, testImgDescr;
	this->extractor->compute( queryKF.img, queryKF.KPts, queryImgDescr );
	this->extractor->compute( testKF.img, testKF.KPts, testImgDescr );

	// match descriptors
	std::vector< DMatch > resultingMatches;
	this->matcher->match(queryImgDescr, testImgDescr, resultingMatches);

	vector<Point2f> matched2DPtsQuery, matched2DPtsTest;
	vector<Eigen::Vector3f> matched3DPtsQuery, matched3DPtsTest;
	//vector<float> measurements;
	Eigen::Vector3f matched3DPtQuery(0.0, 0.0, 0.0), matched3DPtTest(0.0, 0.0, 0.0);

	vector<KeyPoint> matchedQueryKPts, matchedTestKPts;
	for (int i = 0; i < resultingMatches.size(); ++i)	{
		matchedQueryKPts.push_back(queryKF.KPts[resultingMatches[i].queryIdx]);
		matchedTestKPts.push_back(testKF.KPts[resultingMatches[i].trainIdx]);
	}
//	HelperFcts::displayImageKPts("queryImg", queryKF.img, matchedQueryKPts);
//	HelperFcts::displayImageKPts("testImg", testKF.img, matchedTestKPts);
//	HelperFcts::displayMatches("matches", queryKF.img, testKF.img, queryKF.KPts, testKF.KPts, resultingMatches);
//	waitKey(10000);

	for (int i = 0; i < resultingMatches.size(); ++i)	{
		Point2f matched2DPtQuery = queryKF.KPts[resultingMatches[i].queryIdx].pt;
		Point2f matched2DPtTest = testKF.KPts[resultingMatches[i].trainIdx].pt;

		HelperFcts::calc3DPt(matched2DPtQuery, queryKF.fx, queryKF.fy, queryKF.cx, queryKF.cy, queryKF.idepth, matched3DPtQuery);
		//HelperFcts::transform3DPt(queryKF.camToRobot, matched3DPtQuery, matched3DPtQuery);


		HelperFcts::calc3DPt(matched2DPtTest, testKF.fx, testKF.fy, testKF.cx, testKF.cy, testKF.idepth, matched3DPtTest);
		// iGTrafo = pC1C2 = tC2C1
		// iGTrafo = pC2C1 = tC1C2
		Eigen::Matrix4f iGTrafoInv(Eigen::Matrix4f::Identity(4, 4));
		HelperFcts::invTrafo(iGTrafo, iGTrafoInv);
		HelperFcts::transform3DPt(iGTrafoInv, matched3DPtQuery, matched3DPtQuery);


		KalmanFilterScale::Vector u(0);
		KalmanFilterScale::Vector z(3);
		for (int i = 0 ; i < 3 ; ++i) {

			float measurement = matched3DPtTest(i) / matched3DPtQuery(i);
			z(i + 1) = measurement;
		}
		this->filter.step(u, z);
	}
	cout << "scale estimate:	" << this->filter.getX()(1) << endl << "cov:	" << this->filter.calculateP()(1, 1) << endl;
	this->robotScales[queryRID] = this->filter.getX()(1);
}

void CentralStorage::updatePosesOfRobots() {
	cout << "	Update Poses of Robots" << endl;

	// check if any ptcl has update neccessary
	typedef std::map<int, bool> ptClUpdatedMap;
	vector<int> rIDToUpdate;
	for (ptClUpdatedMap::iterator it = this->ptClsInGlobalWorldCOSUpdated.begin(); it != this->ptClsInGlobalWorldCOSUpdated.end(); ++it) {
		if (it->second == false) {
			rIDToUpdate.push_back(it->first);
		}
	}

	if (this->matchedKeyFramesMap.empty()) { // no matches yet
		cout << "		No matches yet" << endl;
		this->pRobotsWorld[1] = Eigen::Matrix4f::Identity(4, 4);

		Eigen::Matrix4f initTrafo(Eigen::Matrix4f::Identity(4, 4));
		initTrafo.block(0, 3, 3, 1) << 0, 0, 0;
		this->pRobotsWorld[2] = initTrafo;
	}
	else if ( !this->matchedKeyFramesMap.empty() && !rIDToUpdate.empty() ) { // there is a new match
		for (int i = 1; i <= this->nbrRobots; ++i) { // iterate through all robots
			if (i == 1) { // update global pose of robot 1
				this->pRobotsWorld[i] = Eigen::Matrix4f::Identity(4, 4);
				this->ptClsInGlobalWorldCOSUpdated[i] = false;
			}
			else { // update global pose of robot 2
				// pRobotsWorld[2] = pRobot2World = tWorldRobot2
				// = tWorldRobot1 * tRobot1Cam1 * tCam1Cam2 * tCam2Robot2
				// = Identity     * pCam1Robot1 * pCam2Cam1 * pRobot2Cam2
				// = Identity     * pCam1Robot1 * pCam2Cam1 * pCam2Robot2^(-1)

				int matchQueryImgCtr = this->matchedKeyFramesMap.rbegin()->second.first;
				int matchTestImgCtr = this->matchedKeyFramesMap.rbegin()->second.second;

				if (this->localPoses[matchQueryImgCtr].first == 1) {
					// R1 = Query, R2 = Test
					Eigen::Matrix4f pCam1Robot1(Eigen::Matrix4f::Identity(4, 4));
					HelperFcts::poseStampedROSToMatrix4f(this->localPoses[matchQueryImgCtr].second, pCam1Robot1);

					Eigen::Matrix4f pCam2Cam1(Eigen::Matrix4f::Identity(4, 4));
					pCam2Cam1 = this->iGMap[matchQueryImgCtr].first;

					Eigen::Matrix4f pRobot2Cam2(Eigen::Matrix4f::Identity(4, 4));
					Eigen::Matrix4f pCam2Robot2(Eigen::Matrix4f::Identity(4, 4));
					HelperFcts::poseStampedROSToMatrix4f(this->localPoses[matchTestImgCtr].second, pCam2Robot2);
					HelperFcts::invTrafo(pCam2Robot2, pRobot2Cam2);
					//this->pRobotsWorld[rID] = Eigen::Matrix4f::Identity(4,4) * pCam1Robot1 * pCam2Cam1 * pRobot2Cam2;

					Eigen::Matrix4f scale(Eigen::Matrix4f::Identity(4, 4));
					scale.block(0, 0, 3, 3) = 1 / this->robotScales[i] * Eigen::Matrix3f::Identity(3, 3);

					Eigen::Matrix4f invTmp;
					HelperFcts::invTrafo(Eigen::Matrix4f::Identity(4, 4) * pCam1Robot1 * pRobot2Cam2 * scale, invTmp);

					this->pRobotsWorld[i] = Eigen::Matrix4f::Identity(4, 4) * pCam1Robot1 * pRobot2Cam2 * scale;
					//this->pRobotsWorld[i] = Eigen::Matrix4f::Identity(4,4) * pCam1Robot1 * pRobot2Cam2 * 1;
					cout << "this->pRobotsWorld[i]: " << this->pRobotsWorld[i] << endl;
				}
				else if (this->localPoses[matchQueryImgCtr].first == 2) {
					// R2 = Query, R1 = Test
					Eigen::Matrix4f pCam1Robot1(Eigen::Matrix4f::Identity(4, 4));
					HelperFcts::poseStampedROSToMatrix4f(this->localPoses[matchTestImgCtr].second, pCam1Robot1);

					Eigen::Matrix4f pCam2Cam1(Eigen::Matrix4f::Identity(4, 4));
					Eigen::Matrix4f pCam1Cam2(Eigen::Matrix4f::Identity(4, 4));
					pCam1Cam2 = this->iGMap[matchQueryImgCtr].first;
					HelperFcts::invTrafo(pCam1Cam2, pCam2Cam1);

					Eigen::Matrix4f pRobot2Cam2(Eigen::Matrix4f::Identity(4, 4));
					Eigen::Matrix4f pCam2Robot2(Eigen::Matrix4f::Identity(4, 4));
					HelperFcts::poseStampedROSToMatrix4f(this->localPoses[matchQueryImgCtr].second, pCam2Robot2);
					HelperFcts::invTrafo(pCam2Robot2, pRobot2Cam2);
					//this->pRobotsWorld[rID] = Eigen::Matrix4f::Identity(4,4) * pCam1Robot1 * pCam2Cam1 * pRobot2Cam2;

					Eigen::Matrix4f scale(Eigen::Matrix4f::Identity(4, 4));
					scale.block(0, 0, 3, 3) = 1 / this->robotScales[i] * Eigen::Matrix3f::Identity(3, 3);

					Eigen::Matrix4f invTmp;
					HelperFcts::invTrafo(Eigen::Matrix4f::Identity(4, 4) * pCam1Robot1 * pRobot2Cam2 * scale, invTmp);

					this->pRobotsWorld[i] = Eigen::Matrix4f::Identity(4, 4) * pCam1Robot1 * pRobot2Cam2 * scale;
					//this->pRobotsWorld[i] = Eigen::Matrix4f::Identity(4,4) * pCam1Robot1 * pRobot2Cam2 * 1;
					cout << "this->pRobotsWorld[i]: " << this->pRobotsWorld[i] << endl;

				}
				this->ptClsInGlobalWorldCOSUpdated[i] = false;
			}
		}
	}
}

void CentralStorage::updatePtCls() {
	cout << "	Update PtCls" << endl;

	/* Naming conventions:
	 * p: pose
	 * t: coordinate transformation
	 * pCOS2COS1:  pose of COS2 relative to COS1 (expressed in COS1)
	 * tCOS2COS1:  coordinate transformation from COS1 to COS2 (transforms representation of 3D coords in COS1 to representation of same 3D points in COS2)
	 * Robot: initial Robot coordinate system (initial pose of robot at initialization of SLAM robot)
	 * Cam: camera coordinate system of corresponding robot (current pose of camera and robot - transformation between current robot COS and camera COS is assumed to be identity)
	 * World: world coordinate system, equivalent to initial robot coordinate system of robot with robot ID rID=1
	 */
	Eigen::Matrix4f pRobotWorld(Eigen::Matrix4f::Identity(4, 4));
	Eigen::Matrix4f pWorldRobot(Eigen::Matrix4f::Identity(4, 4));
	Eigen::Matrix4f tWorldRobot(Eigen::Matrix4f::Identity(4, 4));

	typedef std::map<int, bool> ptClUpdatedMap;
	typedef std::vector<int> rIDToUpdateVec;
	typedef std::map<int, std::pair <int, pcl::PointCloud<pcl::PointXYZ> > > ptClMap;

	// check robot ID for which ptcls can be updated
	vector<int> rIDToUpdate;
	for (ptClUpdatedMap::iterator it = this->ptClsInGlobalWorldCOSUpdated.begin(); it != this->ptClsInGlobalWorldCOSUpdated.end(); ++it) {
		if (it->second == false) {
			cout << "		PtCls to update: " << it->first << endl;
			rIDToUpdate.push_back(it->first);
		}
	}

	if (rIDToUpdate.empty()) { // all ptcls up-to-date
		cout << "		All PtCls up-to-date" << endl;
	}
	else { // update ptcls
		// update all ptcls from all robot IDs in rIDToUpdate
		for (rIDToUpdateVec::iterator itVec = rIDToUpdate.begin(); itVec < rIDToUpdate.end(); ++itVec) {
			// iterate through all robots that need their ptCls to be updated

			for (ptClMap::iterator it = this->ptCls.begin(); it != this->ptCls.end(); ++it) {
				// iterate through all ptcls
				if ( (it->second.first == *itVec) ) {
					// check if the corresponding robot needs update
					this->ptClsInGlobalWorldCOS[it->first].first = it->second.first;
					pRobotWorld = this->pRobotsWorld[*itVec];
					tWorldRobot = pRobotWorld;
					HelperFcts::invTrafo(pRobotWorld, pWorldRobot);
					pcl::transformPointCloud(it->second.second, this->ptClsInGlobalWorldCOS[it->first].second, pRobotWorld);
				}
			}
			this->ptClsInGlobalWorldCOSUpdated[*itVec] = true;
		}
	}
}

void CentralStorage::clearData() {
	this->kFrames.clear();

	this->ptCls.clear();
	this->ptClsInGlobalWorldCOS.clear();
	this->ptClsInGlobalWorldCOSUpdated.clear();

	this->localPoses.clear();
	this->localScales.clear();
	this->robotScales.clear();

	this->pRobotsWorld.clear();
	this->localPoseGraphs.clear();

	globalPoseGraph.clear();

	this->testBOWDescriptors_allRobots.release();
	this->testImgCtr = 0;

	this->matches.clear();
	this->boolMatch = false;
	this->matchedKeyFrames = std::make_pair(-1, -1);
	this->matchedKeyFramesMap.clear();

	this->iGMap.clear();
}
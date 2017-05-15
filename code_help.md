`Pose to matrix 4x4`  HelperFcts::poseStampedROSToMatrix4f




**p**: pose
**t**: coordinate transformation
**pCOS2COS1**:  pose of COS2 relative to COS1 (expressed in COS1)
**tCOS2COS1**:  coordinate transformation from COS1 to COS2 (transforms representation of 3D coords in COS1 to representation of same 3D points in COS2)
**Robot**: initial Robot coordinate system (initial pose of robot at initialization of SLAM robot)
**Cam**: camera coordinate system of corresponding robot (current pose of camera and robot - transformation between current robot COS and camera COS is assumed to be identity)
**World**: world coordinate system, equivalent to initial robot coordinate system of robot with robot ID rID=1



**Eigen::Matrix4f mymatrix** ===>   `float mymatrix[16]`         
**matrix.block(i,j,p,q)**    ===>   `Block of size (p,q), starting at (i,j)`
**pCam1Robot1**             ===>    `pose of robot1 to matrix 4x4`
**pCam1Robot**
`4x4 indentity matrix, pCam1Robot1(4x4)`

**iGMap**
`std::map<int,std::pair <Eigen::Matrix4f, int> >` 
<queryimgnumber, <robotID, pose>
key is the nbr of the queryImg (ImgCtr), iGMap.first: queryImgCtr (queryImgCtr), iGMap.second.first:  IG between 2 keyframes, iGMap.second.second: testImgCtr (testImgCtr of matched kFrame)
it contains the pose of the TestImg (2nd) relative to the QueryImg (1st)

**localPoses**
`std::map<int,std::pair <int,geometry_msgs::PoseStamped> >` 
<queryimgnumber, <robotID, pose>
the key for accessing a local Poses (poses of the keyframe concerning the initial pose of the associated robot)
corresponds to the the kFrames number (ImgCtr), the std::pair contains the associated rID and the actual Transformation
**matchedKeyFramesMap**             
`std::map<int,std::pair <int,int>>` 
(Storage) key is the nbr of the queryImg (ImgCtr)
**pRobotsWorld**                    
`std::map<int,Eigen::Matrix4f>` 
<rID,[matrix]> 
The key for accessing a global poses of the robots (pose of the initial robot COS concerning the world COS) corresponds to the the rID, the map contains the transformation of the associated robot relative to the world COS NOTE: The world COS is identical with the COS of the robot with rID 1 !!!
**ptCls**
`std::map<int,std::pair <int,pcl::PointCloud<pcl::PointXYZ> > >` 
<imgCtr#, <robot ID, point cloud>>
**ptClsInGlobalWorldCOS**  
`std::map<int,std::pair <int,pcl::PointCloud<pcl::PointXYZ> > >` 
<robot ID, <int, point cloud>> 
**ptClsInGlobalWorldCOSUpdated**    
`std::map<int,bool>` 
map key: robot ID rID, 
bool: true if ptcls coming from robot with robot ID rID are updated (ptcls are already transformed to world COS system by transform found after loop closure)
**ptClMap**    
`std::map<int, std::pair <int, pcl::PointCloud<pcl::PointXYZ> > >` 

**robotScales**
`std::map<int,float>` 
<robot ID, Scale>






`Pose to Matrix (4x4)` HelperFcts -> poseStampedROSToMatrix4f (i/p,o/p)
`pointcloud to ros publish` helperPublishPtCl (input, o/p)
                                `use` storage->ptCls
`Publishing assembled local PtCl with robotID`HelperPublishAssembledLocalPtCl
                                (storage, robotID, rospublisher, skipFirstN)
                                `use` storage->ptClsInGlobalWorldCOS

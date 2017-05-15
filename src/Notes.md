#Testing

`Robot1`
#SyncListernerR1 
	- Filtered Point Cloud
	- Keyframe stamped
	- Keyframe graphMsg Stamped


#SLAVEROBOT 
**I/P**
/lsd_slam_r1/graph
/lsd_slam_r1/keyframes

**Outputs**
/slave_robot1/pointscloud2
/slave_robot1/kFMsgStamped
/slave_robot1/kFGraphMsgStamped


#cc_fabmap
`I/p`
/slave_robot1/pointscloud2
/slave_robot1/kFMsgStamped
/slave_robot1/kFGraphMsgStamped

I/p -> syncListener
        syncListenerR1 (id, camerainfo, point cloud, keyframe, keyframegraph, node handler, storage )
            makeSYNCpolicy, camera info, point cloud, keyframe, keyframe graph
            makelocalpointcloud
                `do` 
                original i/p = 0
                create image
                sync
                


`Outputs`

*ine 437 of cc_fabmap_node file - For publishing the points clouds*
ptClCheck1
ptClCheck2
ptClCheck3
ptClCheck4
ptClCheck5

http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29
Transform  (tf Broadcaster)
    - Broadcaster = ba1, br2, br3, br4
    - Transform:    trafoROSMsgR1, tranfoROSMsgR2

if keyframe received
create 
    image
    confusion Matrix
    YML file
    and save the image
Apply
    BOW descriptor
    keyrame to storage
    ptcl to storage
RUN
    FABMAP
    Search for the loop-closure
    Matched
            computer transformation between matched images and between ptcls
            update poses of robots
            update point cloud
            world - cam1
#central_storage
number of robot
number of visual words
test images
stop data collection

SIFT Feature Detector
SIFT Descriptor Extractor
     Descritpor Macher                          *fann based*
BOW Image Descriptor Extractor (Extector + Macher)   `bide`

BOW Trainer (visual words, term criteria, retries number, Kmeans PP centers) `BOWtrainer`     

Kalman Scale Estimation
    for 1 -> number of robots
        pRobotsWorlds                                       - make identity
        point cloud in global world cos updated             -makeisfalse
        robot Scale                                         - 1
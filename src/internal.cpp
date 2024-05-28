#include "internal.h"
#include "transforms.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <natnet_ros_cpp/PointArray.h>
#include <ros/publisher.h>

void Internal::Init(ros::NodeHandle &n)
{
    this->rosparam.getNset(n);
    for (std::size_t i = 0; i < rosparam.object_names.size(); ++i)
    {
        this->tempPointArray.push_back(geometry_msgs::Point());
    }
    for (std::size_t i = 0; i < rosparam.rigidbody_point_names.size(); ++i)
    {
        this->tempPointArray.push_back(geometry_msgs::Point());
    }
}

int Internal::ConnectClient(NatNetClient* g_pClient, sNatNetClientConnectParams &g_connectParams)
{
    // Release previous server
    g_pClient->Disconnect();

    // Init Client and connect to NatNet server
    int retCode = g_pClient->Connect( g_connectParams );
    if (retCode != ErrorCode_OK)
    {
        ROS_ERROR("Unable to connect to server.  Error code: %d. Exiting.", retCode);
        return ErrorCode_Internal;
    }
    else
    {
        // connection succeeded
        void* pResult;
        int nBytes = 0;
        ErrorCode ret = ErrorCode_OK;

        // print server info
        memset( &g_serverDescription, 0, sizeof( g_serverDescription ) );
        ret = g_pClient->GetServerDescription( &g_serverDescription );
        if ( ret != ErrorCode_OK || ! g_serverDescription.HostPresent )
        {
            ROS_ERROR("Unable to connect to server. Host not present. Exiting.");
            return 1;
        }
        

        // get mocap frame rate
        ret = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            float fRate = *((float*)pResult);
            ROS_INFO("Mocap Framerate : %3.2f", fRate);
        }
        else
            ROS_ERROR("Error getting frame rate.");

        // get # of analog samples per mocap frame of data
        ret = g_pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            //g_analogSamplesPerMocapFrame = *((int*)pResult);
            ROS_INFO("Analog Samples Per Mocap Frame : %d", *((int*)pResult));
        }
        else
            ROS_ERROR("Error getting Analog frame rate.");
    }

    return ErrorCode_OK;
}

void Internal::MessageHandler( Verbosity msgType, const char* msg )
{
    // Optional: Filter out debug messages
    if ( msgType < Verbosity_Info )
    {
        return;
    }

    printf( "\n[NatNetLib]" );
    switch ( msgType )
    {
        case Verbosity_Debug:
            printf( " [DEBUG]" );
            break;
        case Verbosity_Info:
            printf( "  [INFO]" );
            break;
        case Verbosity_Warning:
            printf( "  [WARN]" );
            break;
        case Verbosity_Error:
            printf( " [ERROR]" );
            break;
        default:
            printf( " [?????]" );
            break;
    }

    printf( ": %s", msg );
}

void Internal::Info(NatNetClient* g_pClient, ros::NodeHandle &n, Internal &internal)
{
    ros::NodeHandle thisN;
    ROS_INFO("[SampleClient] Requesting Data Descriptions...");
    sDataDescriptions* pDataDefs = NULL;
    int iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
    if (iResult != ErrorCode_OK || pDataDefs == NULL)
    {
        ROS_INFO("[SampleClient] Unable to retrieve Data Descriptions.");
    }
    else
    {
        ROS_INFO("[SampleClient] Received %d Data/Devices Descriptions:", pDataDefs->nDataDescriptions );

        int markers = 0;
        for(int i=pDataDefs->nDataDescriptions - 1; i >= 0; i--)
        {
            if(pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
            {
                // RigidBody
                sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
                ROS_INFO("RigidBody found : %s", pRB->szName);
                ROS_INFO_COND(rosparam.log_internals, "RigidBody ID : %d", pRB->ID);
                ROS_INFO_COND(rosparam.log_internals, "RigidBody Parent ID : %d", pRB->parentID);
                ROS_INFO_COND(rosparam.log_internals, "Parent Offset : %3.2f,%3.2f,%3.2f", pRB->offsetx, pRB->offsety, pRB->offsetz);
                
                // Creating publisher for the rigid bodies if found any
                std::string body_name(pRB->szName);
                if(rosparam.pub_rigid_body)
                {
                    this->ListRigidBodies[pRB->ID] = body_name;
                    ROS_INFO("Body ID : %d", pRB->ID);
                    this->RigidbodyPub[pRB->szName] = thisN.advertise<geometry_msgs::PoseStamped>("rigidbodies/" + std::to_string(pRB->ID) + "/pose", 50);
                }
                if ( pRB->MarkerPositions != NULL && pRB->MarkerRequiredLabels != NULL )
                {
                    for ( int markerIdx = 0; markerIdx < pRB->nMarkers; ++markerIdx )
                    {
                        const MarkerData& markerPosition = pRB->MarkerPositions[markerIdx];
                        const int markerRequiredLabel = pRB->MarkerRequiredLabels[markerIdx];
                        // Creating publisher for the markers of the rigid bodies
                        if(rosparam.pub_rigid_body_marker)
                            this->RigidbodyMarkerPub[std::to_string(pRB->ID)+std::to_string(markerIdx+1)] = thisN.advertise<geometry_msgs::PointStamped>(rosparam.rigidbody_point_names[markers]+"/point", 50);
                        ROS_INFO_COND(rosparam.log_internals,  "\tMarker #%d:", markerIdx );
                        ROS_INFO_COND(rosparam.log_internals,  "\t\tPosition: %.2f, %.2f, %.2f", markerPosition[0], markerPosition[1], markerPosition[2] );

                        if ( markerRequiredLabel != 0 )
                        {
                            ROS_INFO_COND(rosparam.log_internals,  "\t\tRequired active label: %d", markerRequiredLabel );
                        }
                        markers++;
                    }
                }
            }
            else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Camera)
            {
                // Camera
                sCameraDescription* pCamera = pDataDefs->arrDataDescriptions[i].Data.CameraDescription;
                ROS_INFO_COND(rosparam.log_internals, "Camera Name : %s", pCamera->strName);
                ROS_INFO_COND(rosparam.log_internals, "Camera Position (%3.2f, %3.2f, %3.2f)", pCamera->x, pCamera->y, pCamera->z);
                ROS_INFO_COND(rosparam.log_internals, "Camera Orientation (%3.2f, %3.2f, %3.2f, %3.2f)", pCamera->qx, pCamera->qy, pCamera->qz, pCamera->qw);
            }
            else
            {
                ROS_WARN_COND(rosparam.log_internals, "Unknown data type detected.");
                // Unknown
            }
        }
        if (rosparam.pub_pointcloud)
        {
            this->PointcloudPub = thisN.advertise<sensor_msgs::PointCloud>("pointcloud",50);
        }
        if (rosparam.pub_individual_marker)
        {
            for (int i=0; i<(int) rosparam.object_list.size(); i++)
            {
                this->IndividualMarkerPub[rosparam.object_list[i].name] = thisN.advertise<geometry_msgs::PointStamped>(rosparam.object_list[i].name + "/point", 50);
            }
            this->PointArrayPub = thisN.advertise<natnet_ros_cpp::PointArray>("MocapPointArray", 1);
        }
    }
}

void Internal::LatenciInfo(sFrameOfMocapData* data, void* pUserData, Internal &internal)
{
    NatNetClient* pClient = (NatNetClient*) pUserData;
    // This figure may appear slightly higher than the "software latency" reported in the Motive user interface,
    // because it additionally includes the time spent preparing to stream the data via NatNet.
    const uint64_t softwareLatencyHostTicks = data->TransmitTimestamp - data->CameraDataReceivedTimestamp;
    const double softwareLatencyMillisec = (softwareLatencyHostTicks * 1000) / static_cast<double>(internal.g_serverDescription.HighResClockFrequency);
    // Transit latency is defined as the span of time between Motive transmitting the frame of data, and its reception by the client (now).
    // The SecondsSinceHostTimestamp method relies on NatNetClient's internal clock synchronization with the server using Cristian's algorithm.
    const double transitLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->TransmitTimestamp ) * 1000.0;
    ROS_INFO_COND(internal.rosparam.log_latencies, "Software latency : %.2lf milliseconds", softwareLatencyMillisec);
    // If it's unavailable (for example, with USB camera systems, or during playback), this field will be zero.
    const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;
    if ( bSystemLatencyAvailable )
    {
        const uint64_t systemLatencyHostTicks = data->TransmitTimestamp - data->CameraMidExposureTimestamp;
        const double systemLatencyMillisec = (systemLatencyHostTicks * 1000) / static_cast<double>(internal.g_serverDescription.HighResClockFrequency);
        
        const double clientLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->CameraMidExposureTimestamp ) * 1000.0;
        ROS_INFO_COND(internal.rosparam.log_latencies,  "System latency : %.2lf milliseconds", systemLatencyMillisec );
        ROS_INFO_COND(internal.rosparam.log_latencies,  "Total client latency : %.2lf milliseconds (transit time +%.2lf ms)", clientLatencyMillisec, transitLatencyMillisec );
    }
    else
    {
        ROS_INFO_COND(internal.rosparam.log_latencies,  "Transit latency : %.2lf milliseconds", transitLatencyMillisec );
    }
}

void Internal::DataHandler(sFrameOfMocapData* data, void* pUserData, Internal &internal)
{   
    int i=0;
    ROS_INFO_COND(internal.rosparam.log_frames, "FrameID : %d", data->iFrame);

    // Rigid Bodies
    ROS_INFO_COND(internal.rosparam.log_frames, "Rigid Bodies [Count=%d]", data->nRigidBodies);
    for(i=0; i < data->nRigidBodies; i++)
        {
            if(internal.rosparam.pub_rigid_body)
            {
                PubRigidbodyPose(data->RigidBodies[i], internal);
            }
        ROS_INFO_COND(internal.rosparam.log_frames, "Rigid Body [ID=%d  Error=%3.2f]", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError);//, bTrackingValid);
        ROS_INFO_COND(internal.rosparam.log_frames, "x\ty\tz\tqx\tqy\tqz\tqw");
        ROS_INFO_COND(internal.rosparam.log_frames, "%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f",
            data->RigidBodies[i].x, data->RigidBodies[i].y, data->RigidBodies[i].z,
            data->RigidBodies[i].qx, data->RigidBodies[i].qy, data->RigidBodies[i].qz, data->RigidBodies[i].qw);
        }

    // Markers
    for(i=0; i < data->nLabeledMarkers; i++) 
    {  
        ROS_INFO_COND(internal.rosparam.log_frames, "Markers [Count=%i]", i);
        ROS_INFO_COND(internal.rosparam.log_frames, "x\ty\tz");
        ROS_INFO_COND(internal.rosparam.log_frames, "%3.2f\t%3.2f\t%3.2f", data->LabeledMarkers[i].x, data->LabeledMarkers[i].y, data->LabeledMarkers[i].z);
        bool bUnlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
        if(internal.rosparam.pub_individual_marker && bUnlabeled)
        {   
            PubMarkerPose(data->LabeledMarkers[i], internal);
        }
        if (internal.rosparam.pub_pointcloud)
        {
            PubPointCloud(data->LabeledMarkers[i], internal);
        }
        if(internal.rosparam.pub_rigid_body_marker && !bUnlabeled)
        {
            PubRigidbodyMarker(data->LabeledMarkers[i], internal);
        }
    }
    if (internal.rosparam.pub_individual_marker)
    {
        internal.rosparam.object_list_prev = internal.rosparam.object_list;
        if(internal.UnlabeledCount < (int)internal.rosparam.object_list.size() && internal.rosparam.error_amp==1.0)
            internal.rosparam.error_amp = internal.rosparam.error_amp*2;
        else   
            internal.rosparam.error_amp = 1.0;

        internal.msgPointcloud.points.clear();
        internal.UnlabeledCount = 0;

        natnet_ros_cpp::PointArray pointArray;
        pointArray.header.stamp = ros::Time::now();
        pointArray.header.frame_id = "base";
        pointArray.points = internal.tempPointArray;
        internal.PointArrayPub.publish(pointArray);
    }
    if(internal.rosparam.pub_pointcloud)
    {
        internal.msgPointcloud.header.frame_id= internal.rosparam.globalFrame;
        internal.msgPointcloud.header.stamp=ros::Time::now();
        internal.PointcloudPub.publish(internal.msgPointcloud);
    }
}

void Internal::PubRigidbodyPose(sRigidBodyData &data, Internal &internal)
{
    // Creating a msg to put data related to the rigid body and 
    geometry_msgs::PoseStamped msgRigidBodyPose;
    msgRigidBodyPose.header.stamp = ros::Time::now();
    msgRigidBodyPose.header.frame_id = "base";
    msgRigidBodyPose.pose.position.x = data.x;
    msgRigidBodyPose.pose.position.y = data.y;
    msgRigidBodyPose.pose.position.z = data.z;
    msgRigidBodyPose.pose.orientation.x = data.qx;
    msgRigidBodyPose.pose.orientation.y = data.qy;
    msgRigidBodyPose.pose.orientation.z = data.qz;
    msgRigidBodyPose.pose.orientation.w = data.qw;

    // Transforming position from "optitrack" frame to "base" frame
    tf2::doTransform(msgRigidBodyPose, msgRigidBodyPose, transforms::optitrackToBase);

    internal.RigidbodyPub[internal.ListRigidBodies[data.ID]].publish(msgRigidBodyPose);
    
    // creating tf frame to visualize in the rviz
    static tf2_ros::TransformBroadcaster tfRigidBodies;
    geometry_msgs::TransformStamped msgTFRigidBodies;
    msgTFRigidBodies.header.stamp = ros::Time::now();
    msgTFRigidBodies.header.frame_id = "base";
    msgTFRigidBodies.child_frame_id = "rigidbodies_" + std::to_string(data.ID) + "_pose";
    msgTFRigidBodies.transform.translation.x = msgRigidBodyPose.pose.position.x;
    msgTFRigidBodies.transform.translation.y = msgRigidBodyPose.pose.position.y;
    msgTFRigidBodies.transform.translation.z = msgRigidBodyPose.pose.position.z;
    msgTFRigidBodies.transform.rotation.x = msgRigidBodyPose.pose.orientation.x;
    msgTFRigidBodies.transform.rotation.y = msgRigidBodyPose.pose.orientation.y;
    msgTFRigidBodies.transform.rotation.z = msgRigidBodyPose.pose.orientation.z;
    msgTFRigidBodies.transform.rotation.w = msgRigidBodyPose.pose.orientation.w;
    tfRigidBodies.sendTransform(msgTFRigidBodies);
}

void Internal::PubMarkerPose(sMarker &data, Internal &internal)
{   
    int update = nn_filter(internal.rosparam.object_list, data, internal.rosparam.E,  internal.rosparam.E_x, internal.rosparam.E_y, internal.rosparam.E_z, internal.rosparam.individual_error, internal.rosparam.error_amp);
    if (update>=0)
    {   
        internal.UnlabeledCount+=1;
        internal.rosparam.object_list[update].detected = true;
        internal.rosparam.object_list[update].x = data.x;
        internal.rosparam.object_list[update].y = data.y;
        internal.rosparam.object_list[update].z = data.z;
    
        geometry_msgs::PointStamped msgMarkerPoint;
        msgMarkerPoint.header.stamp = ros::Time::now();
        msgMarkerPoint.header.frame_id = "base";
        msgMarkerPoint.point.x = data.x;
        msgMarkerPoint.point.y = data.y;
        msgMarkerPoint.point.z = data.z;

        // Transforming position from "optitrack" frame to "base" frame
        tf2::doTransform(msgMarkerPoint, msgMarkerPoint, transforms::optitrackToBase);
        internal.tempPointArray[static_cast<int>(internal.rosparam.rigidbody_point_names.size()) + update] = msgMarkerPoint.point;
        internal.IndividualMarkerPub[internal.rosparam.object_list[update].name].publish(msgMarkerPoint);
    }
}

void Internal::PubPointCloud(sMarker &data, Internal &internal)
{

    geometry_msgs::Point32 msgPoint;
    msgPoint.x = data.x;
    msgPoint.y = data.y;
    msgPoint.z = data.z;
    internal.msgPointcloud.points.push_back(msgPoint);
}

void Internal::PubRigidbodyMarker(sMarker &data, Internal &internal)
{
    int modelID, markerID;
    NatNet_DecodeID( data.ID, &modelID, &markerID );

    geometry_msgs::PointStamped msgMarkerPoint;
    msgMarkerPoint.header.stamp = ros::Time::now();
    msgMarkerPoint.header.frame_id = "base";
    msgMarkerPoint.point.x = data.x + transforms::optitrackToBase.transform.translation.x;
    msgMarkerPoint.point.y = data.y + transforms::optitrackToBase.transform.translation.y;
    msgMarkerPoint.point.z = data.z + transforms::optitrackToBase.transform.translation.z;

    int markerNum = 0;
    for (int i = 0; i < modelID - 1; i++) {
        markerNum += internal.rosparam.rigidbody_points_per_body[i];
    }
    internal.tempPointArray[markerNum + markerID - 1] = msgMarkerPoint.point;
    internal.RigidbodyMarkerPub[std::to_string(modelID)+std::to_string(markerID)].publish(msgMarkerPoint);
}
geometry_msgs::Pose convertToPoseMsg(const Grasp &hand)
{
    gpd_fix::GraspConfig msg;
    tf::pointEigenToMsg(hand.getGraspBottom(), msg.bottom);
    tf::pointEigenToMsg(hand.getGraspTop(), msg.top);
    tf::pointEigenToMsg(hand.getGraspSurface(), msg.surface);
    tf::vectorEigenToMsg(hand.getApproach(), msg.approach);
    tf::vectorEigenToMsg(hand.getBinormal(), msg.binormal);
    tf::vectorEigenToMsg(hand.getAxis(), msg.axis);
    msg.width.data = hand.getGraspWidth();
    msg.score.data = hand.getScore();
    tf::pointEigenToMsg(hand.getSample(), msg.sample);

    Eigen::Vector3d base = hand.getGraspBottom() - 0.00 * hand.getApproach();
    geometry_msgs::Point base_point;
    tf::pointEigenToMsg(base, base_point);
    geometry_msgs::Point position = base_point;
    geometry_msgs::Quaternion orientation;
    Eigen::Quaterniond quat(hand.getFrame());
    orientation.x = quat.x();
    orientation.y = quat.y();
    orientation.z = quat.z();
    orientation.w = quat.w();

    geometry_msgs::Pose pose;
    pose.position = position;
    pose.orientation = orientation;

    return pose;
}

void transform_gpd_to_moveit(const geometry_msgs::Pose &effector_pose_in, geometry_msgs::Pose &effector_pose_out)
{
    effector_pose_out = effector_pose_in;

    // check the Z aix diretcion
    //  vz = (0,0,1); vz = tf * vz; if vz(3) > 0, rotate pi
    tf::Pose normal_vector_z = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 1.0));
    // generate a direction only transform
    tf::Pose trandsform_tf;
    geometry_msgs::Pose pose_trans = effector_pose_out;
    pose_trans.position.x = 0.0;
    pose_trans.position.y = 0.0;
    pose_trans.position.z = 0.0;
    tf::poseMsgToTF(pose_trans, trandsform_tf);
    normal_vector_z = trandsform_tf * normal_vector_z;
    if (normal_vector_z.getOrigin().z() < 0) // the z axis if downside, rotate it
    {
        tf::Quaternion quat_x180 = tf::createQuaternionFromRPY(3.14159, 0, 0);
        tf::Transform transform_x180 = tf::Transform(quat_x180, tf::Vector3(0.0, 0.0, 0.0));
        trandsform_tf = trandsform_tf * transform_x180;
        tf::poseTFToMsg(trandsform_tf, pose_trans);
        effector_pose_out.orientation = pose_trans.orientation; // update the pose
    }

    // change the coordinates definition to moveit pose
    // static tranform
    tf::Transform transform;
    tf::Quaternion quat = tf::createQuaternionFromRPY(0, -1.5707, 3.14159);
    transform.setRotation(quat);
    // transform
    tf::Pose pose_tf;
    tf::poseMsgToTF(effector_pose_out, pose_tf);
    pose_tf = pose_tf * transform; // transform
    // tranformed pose
    geometry_msgs::Pose pose_tranformed;
    tf::poseTFToMsg(pose_tf, pose_tranformed);
    pose_tranformed.position = effector_pose_out.position; // we only need the axis tranformed, the position keeps same

    effector_pose_out = pose_tranformed;
}
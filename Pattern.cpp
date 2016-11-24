#include "Pattern.hpp"
#include "CameraInfo.hpp"

void PatternTrackingInfo::computePose(const Pattern& pattern, const CameraInfo& camera)
{
  cv::Mat Rvec;
  cv::Mat_<float> Tvec;
  cv::Mat raux,taux;

  cv::solvePnP(pattern.points3d, points2d, camera.getIntrinsic(), camera.getDistorsion(),raux,taux);
  raux.convertTo(Rvec,CV_32F);
  taux.convertTo(Tvec ,CV_32F);

  cv::Mat_<float> rotMat(3,3); 
  cv::Rodrigues(Rvec, rotMat);

  // Copy to transformation matrix
  for (int col=0; col<3; col++)
  {
    for (int row=0; row<3; row++)
    {        
     pose3d.r().mat[row][col] = rotMat(row,col); // Copy rotation component
    }
    pose3d.t().data[col] = Tvec(col); // Copy translation component
  }

  // Since solvePnP finds camera location, w.r.t to marker pose, to get marker pose w.r.t to the camera we invert it.
  pose3d = pose3d.getInverted();
}

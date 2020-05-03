#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

namespace cv{
namespace aruco{

using namespace std;

static void _getSingleMarkerObjectPoints(float markerLength, OutputArray _objPoints) {

    CV_Assert(markerLength > 0);

    _objPoints.create(4, 1, CV_32FC3);
    Mat objPoints = _objPoints.getMat();
    // set coordinate system in the middle of the marker, with Z pointing out
    objPoints.ptr< Vec3f >(0)[0] = Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr< Vec3f >(0)[1] = Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr< Vec3f >(0)[2] = Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints.ptr< Vec3f >(0)[3] = Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);
}


/**
  * ParallelLoopBody class for the parallelization of the single markers pose estimation
  * Called from function estimatePoseSingleMarkers()
  */

class SinglePoseEstimationParallelParam : public ParallelLoopBody {
    public:
    SinglePoseEstimationParallelParam(Mat& _markerObjPoints, InputArrayOfArrays _corners,
                                 InputArray _cameraMatrix, InputArray _distCoeffs,
                                 Mat& _rvecs, Mat& _tvecs, bool _useEG, int _PnPflags)
        : markerObjPoints(_markerObjPoints), corners(_corners), cameraMatrix(_cameraMatrix),
          distCoeffs(_distCoeffs), rvecs(_rvecs), tvecs(_tvecs), useEG(_useEG), PnPflags(_PnPflags){}

    void operator()(const Range &range) const{
        const int begin = range.start;
        const int end = range.end;

        for(int i = begin; i < end; i++) {
            solvePnP(markerObjPoints, corners.getMat(i), cameraMatrix, distCoeffs,
                    rvecs.at<Vec3d>(i), tvecs.at<Vec3d>(i), useEG, PnPflags);
        }
    }

    private:
    SinglePoseEstimationParallelParam &operator=(const SinglePoseEstimationParallelParam &);

    Mat& markerObjPoints;
    InputArrayOfArrays corners;
    InputArray cameraMatrix, distCoeffs;
    Mat& rvecs, tvecs;
    bool useEG;
    int PnPflags;
};


/**
  */
void estimatePoseSingleMarkersParam(InputArrayOfArrays _corners, float markerLength,
                               InputArray _cameraMatrix, InputArray _distCoeffs,
                               OutputArray _rvecs, OutputArray _tvecs, OutputArray _objPoints, bool _useEG, int _PnPflags) {

    CV_Assert(markerLength > 0);

    Mat markerObjPoints;
    _getSingleMarkerObjectPoints(markerLength, markerObjPoints);
    int nMarkers = (int)_corners.total();
    _rvecs.create(nMarkers, 1, CV_64FC3);
    _tvecs.create(nMarkers, 1, CV_64FC3);

    Mat rvecs = _rvecs.getMat(), tvecs = _tvecs.getMat();

    parallel_for_(Range(0, nMarkers),
                  SinglePoseEstimationParallelParam(markerObjPoints, _corners, _cameraMatrix,
                                               _distCoeffs, rvecs, tvecs, _useEG, _PnPflags));
    if(_objPoints.needed()){
        markerObjPoints.convertTo(_objPoints, -1);
    }
}

}}
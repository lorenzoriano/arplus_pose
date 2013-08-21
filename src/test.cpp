#include <ARToolKitPlus/TrackerSingleMarker.h>
#include <iostream>

int main() {

    ARToolKitPlus::TrackerSingleMarker tracker(640, 480);
    tracker.init(NULL, 0, 0);
    ARToolKitPlus::Camera* camera = new ARToolKitPlus::Camera();
    camera->xsize = 640;
    camera->ysize = 480;
    camera->mat[0][0] = 596.28278;
    camera->mat[1][0] = 0;
    camera->mat[2][0] = 0;
    camera->mat[0][1] = 0;
    camera->mat[1][1] = 598.22026;
    camera->mat[2][1] = 0;
    camera->mat[0][2] = 683.95141;
    camera->mat[1][2] = 450.66728;
    camera->mat[2][2] = 1.0;
    camera->mat[0][3] = 0;
    camera->mat[1][3] = 0;
    camera->mat[2][3] = 0;

    camera->kc[0] = -0.27569944;
    camera->kc[1] = 0.09206397200000001;
    camera->kc[2] =  0.0001978362;
    camera->kc[3] = 0.00034858682;
    camera->kc[4] = -0.014973806000000001;

    camera->fc[0] = camera->mat[0][0];
    camera->fc[1] = camera->mat[1][1];
    camera->cc[0] = camera->mat[0][2];
    camera->cc[1] = camera->mat[1][2];

    tracker.setCamera(camera, 0., 1000.);

    tracker.getCamera()->printSettings();
//    camera->printSettings();


}


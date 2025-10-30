import numpy as np
from dt_apriltags import Detector
import cv2

class WorkspaceExtrinsic:
    def __init__(self, tagFamily, squareSize, intrinsicsPath):
        self.tagFamily = tagFamily

        self.square_size = squareSize

        self.tagCorners = np.array([
            [-self.square_size/2, self.square_size/2, 0],
            [self.square_size/2, self.square_size/2, 0],
            [self.square_size/2, -self.square_size/2, 0],
            [-self.square_size/2, -self.square_size/2, 0]], dtype=np.float32)
        
        #load intrinsics
        self.data = np.load(intrinsicsPath)
        self.K = self.data['camera_matrix']
        self.dist = self.data['dist_coeffs']



        self.at_detector = Detector(
            families=self.tagFamily,
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25,
            debug=False,
        )
    
    def readImageGray(Self, image):
        if isinstance(image, str): #if image path
            img = cv2.imread(image)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            img = cv2.imread(image)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        return gray

    def detectTags(self, gray):
        detection = self.at_detector.detect(gray)

        return detection
    
    def workspacePose(self, image):
        gray = self.readImageGray(image)

        detection = self.detectTags(gray)

        imgpoints = np.array(
            [
                detection[2].center,
                detection[3].center,
                detection[0].center,
                detection[1].center,    
            ])

        objpoints = self.tagCorners

        retval, rvec_workspace, tvec_workspace = cv2.solvePnP(objpoints, imgpoints, self.K, self.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)

        return rvec_workspace, tvec_workspace

    def cameraPose(self, rvec_workspace, tvec_workspace):
        # Convert rvec to rotation matrix
        R_workspace2cam, _ = cv2.Rodrigues(rvec_workspace)

        # Invert the rotation
        R_cam2workspace = R_workspace2cam.T

        # Invert the translation
        tvec_cam = -R_cam2workspace @ tvec_workspace

        # Convert rotation matrix back to rvec
        rvec_cam, _ = cv2.Rodrigues(R_cam2workspace)

        return rvec_cam, tvec_cam

if __name__ == '__main__':
    image_path = "calib_images/workspace.png"

    workspaceExtrinsic = WorkspaceExtrinsic("tagStandard41h12", 0.6, 'config/camera_calib.npz')

    rvecWorkspace, tvecWorkspace = workspaceExtrinsic.workspacePose(image_path)
    revecCam, tvecCam = workspaceExtrinsic.cameraPose(rvecWorkspace, tvecWorkspace)

    print("workspace pose found for", image_path)
    print(rvecWorkspace.ravel())
    print(tvecWorkspace.ravel())

    print("camera pose found")
    print(revecCam.ravel())
    print(tvecCam.ravel())
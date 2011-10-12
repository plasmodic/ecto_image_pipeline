from ecto_opencv.calib import PatternDetector, PatternDrawer, CameraCalibrator, ASYMMETRIC_CIRCLES_GRID,CHESSBOARD, GatherPoints

def create_detector_drawer_circles():
    rows = 5
    cols = 3
    square_size = 0.04 #4 cm
    pattern_type = ASYMMETRIC_CIRCLES_GRID
    return (PatternDetector(rows=rows, cols=cols,
                                  pattern_type=pattern_type, square_size=square_size), PatternDrawer(rows=rows, cols=cols))


def create_detector_drawer_chessboard():
    rows = 7
    cols = 6
    square_size = 0.108 #108 mmc
    pattern_type = CHESSBOARD
    return (PatternDetector(rows=rows, cols=cols,
                                  pattern_type=pattern_type, square_size=square_size), PatternDrawer(rows=rows, cols=cols))

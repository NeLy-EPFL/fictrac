/// Elevation Measurement Tool
/// \brief      Interactive tool for measuring angles relative to a circle's center.
/// \author     Adapted from FicTrac ConfigGui by Richard Moore

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <memory>

using cv::Mat;
using cv::Point2d;
using cv::Scalar;
using std::vector;
using std::string;

// Constants
const int ZOOM_DIM = 600;
const double ZOOM_SCL = 1.0 / 10.0;
const double PI = 3.14159265358979323846;

// Application modes
enum AppMode {
    MODE_CIRCLE_PTS,    // Annotating circle points
    MODE_CIRCLE_DONE,   // Circle confirmed, ready for angle measurement
    MODE_ANGLE_PT       // Measuring angle to a point
};

// Reference axis for angle measurement
enum RefAxis {
    AXIS_POS_X = 0,     // +X (right)
    AXIS_POS_Y = 1,     // +Y (up in image coords, but we display as "up")
    AXIS_NEG_X = 2,     // -X (left)
    AXIS_NEG_Y = 3      // -Y (down)
};

const char* AXIS_NAMES[] = {"+X", "+Y", "-X", "-Y"};

// Input data structure (similar to ConfigGui)
struct InputData {
    bool newEvent;
    Point2d cursorPt;
    vector<Point2d> circPts;
    Point2d anglePt;
    bool anglePtSet;
    AppMode mode;
    float ptScl;
};

// Circle parameters
struct Circle {
    Point2d center;
    double radius;
    bool valid;
};

// Clamp helper
template<typename T>
T clamp(T val, T minVal, T maxVal) {
    return std::max(minVal, std::min(val, maxVal));
}

/// Fit circle to 2D points using algebraic least squares
/// Circle equation: x² + y² + Dx + Ey + F = 0
/// Center = (-D/2, -E/2), Radius = sqrt(D²/4 + E²/4 - F)
/// Returns true if fit was successful (need at least 3 points)
bool fitCircle2D(const vector<Point2d>& pts, Circle& circle) {
    size_t n = pts.size();
    if (n < 3) {
        circle.valid = false;
        return false;
    }

    // Compute sums for the 3x3 linear system
    double sumX = 0, sumY = 0;
    double sumX2 = 0, sumY2 = 0, sumXY = 0;
    double sumX3 = 0, sumY3 = 0;
    double sumX2Y = 0, sumXY2 = 0;
    double sumZ = 0;      // sum of (x² + y²)
    double sumXZ = 0;     // sum of x*(x² + y²)
    double sumYZ = 0;     // sum of y*(x² + y²)

    for (const auto& p : pts) {
        double x = p.x, y = p.y;
        double x2 = x * x, y2 = y * y;
        double z = x2 + y2;  // x² + y²

        sumX += x;
        sumY += y;
        sumX2 += x2;
        sumY2 += y2;
        sumXY += x * y;
        sumZ += z;
        sumXZ += x * z;
        sumYZ += y * z;
    }

    // Set up the 3x3 system: A * [D, E, F]^T = B
    // | sumX²  sumXY  sumX |   | D |   | -sumXZ |
    // | sumXY  sumY²  sumY | * | E | = | -sumYZ |
    // | sumX   sumY   n    |   | F |   | -sumZ  |

    cv::Mat A = (cv::Mat_<double>(3, 3) <<
        sumX2, sumXY, sumX,
        sumXY, sumY2, sumY,
        sumX,  sumY,  static_cast<double>(n));

    cv::Mat B = (cv::Mat_<double>(3, 1) <<
        -sumXZ,
        -sumYZ,
        -sumZ);

    cv::Mat X;
    if (!cv::solve(A, B, X, cv::DECOMP_LU)) {
        circle.valid = false;
        return false;
    }

    double D = X.at<double>(0);
    double E = X.at<double>(1);
    double F = X.at<double>(2);

    circle.center.x = -D / 2.0;
    circle.center.y = -E / 2.0;
    double r2 = D * D / 4.0 + E * E / 4.0 - F;

    if (r2 < 0) {
        circle.valid = false;
        return false;
    }

    circle.radius = std::sqrt(r2);
    circle.valid = true;
    return true;
}

/// Calculate angle from a point to the circle center
/// Returns angle in degrees, measured from the specified reference axis, counterclockwise
/// refAxis: 0 = +X, 1 = +Y, 2 = -X, 3 = -Y
double calculateAngle(const Point2d& pt, const Circle& circle, int refAxis = AXIS_POS_X) {
    double dx = pt.x - circle.center.x;
    double dy = pt.y - circle.center.y;
    double angle = std::atan2(-dy, dx) * 180.0 / PI;  // Negate dy because image Y is down

    // Adjust for reference axis (each axis is 90 degrees CCW from the previous)
    angle -= refAxis * 90.0;

    // Normalize to [-180, 180]
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;

    return angle;
}

/// Mouse callback function
void onMouseEvent(int event, int x, int y, int flags, void* ptr) {
    InputData* pdata = static_cast<InputData*>(ptr);

    if (pdata->ptScl > 0) {
        x = static_cast<int>(std::round(x * pdata->ptScl));
        y = static_cast<int>(std::round(y * pdata->ptScl));
    }

    switch (event) {
        case cv::EVENT_LBUTTONUP:
            if (pdata->mode == MODE_CIRCLE_PTS) {
                pdata->circPts.push_back(Point2d(x, y));
                pdata->newEvent = true;
            } else if (pdata->mode == MODE_ANGLE_PT) {
                pdata->anglePt = Point2d(x, y);
                pdata->anglePtSet = true;
                pdata->newEvent = true;
            }
            break;

        case cv::EVENT_RBUTTONUP:
            if (pdata->mode == MODE_CIRCLE_PTS) {
                if (!pdata->circPts.empty()) {
                    pdata->circPts.pop_back();
                }
                pdata->newEvent = true;
            } else if (pdata->mode == MODE_ANGLE_PT) {
                pdata->anglePtSet = false;
                pdata->newEvent = true;
            }
            break;

        case cv::EVENT_MOUSEMOVE:
            pdata->cursorPt.x = x;
            pdata->cursorPt.y = y;
            break;

        default:
            break;
    }
}

/// Create zoomed ROI around cursor
void createZoomROI(Mat& zoom_roi, const Mat& frame, const Point2d& pt, int orig_dim) {
    int x = frame.cols / 2;
    if (pt.x >= 0) {
        x = clamp(static_cast<int>(pt.x - orig_dim / 2 + 0.5), orig_dim / 2, frame.cols - 1 - orig_dim);
    }
    int y = frame.rows / 2;
    if (pt.y >= 0) {
        y = clamp(static_cast<int>(pt.y - orig_dim / 2 + 0.5), 0, frame.rows - 1 - orig_dim);
    }

    // Ensure we don't go out of bounds
    x = clamp(x, 0, frame.cols - orig_dim);
    y = clamp(y, 0, frame.rows - orig_dim);

    Mat crop_rect = frame(cv::Rect(x, y, orig_dim, orig_dim));
    cv::resize(crop_rect, zoom_roi, zoom_roi.size());
}

/// Draw cursor crosshair
void drawCursor(Mat& img, const Point2d& pt, Scalar colour) {
    const int inner_rad = std::max(static_cast<int>(img.cols / 500.0 + 0.5), 2);
    const int outer_rad = std::max(static_cast<int>(img.cols / 150.0 + 0.5), 5);

    cv::line(img, pt - Point2d(outer_rad, outer_rad), pt - Point2d(inner_rad, inner_rad), colour, 1, cv::LINE_AA);
    cv::line(img, pt + Point2d(inner_rad, inner_rad), pt + Point2d(outer_rad, outer_rad), colour, 1, cv::LINE_AA);
    cv::line(img, pt - Point2d(-outer_rad, outer_rad), pt - Point2d(-inner_rad, inner_rad), colour, 1, cv::LINE_AA);
    cv::line(img, pt + Point2d(-inner_rad, inner_rad), pt + Point2d(-outer_rad, outer_rad), colour, 1, cv::LINE_AA);
}

/// Draw a 2D circle
void drawCircle2D(Mat& img, const Circle& circle, Scalar colour, bool solid = true) {
    if (!circle.valid) return;

    int thickness = solid ? 1 : 1;  // Thinner so purple arc can overlay it
    cv::circle(img, circle.center, static_cast<int>(circle.radius), colour, thickness, cv::LINE_AA);

    // Draw center point
    cv::circle(img, circle.center, 4, colour, -1, cv::LINE_AA);
}

/// Draw line from point to circle center with angle annotation
/// refAxis: 0 = +X, 1 = +Y, 2 = -X, 3 = -Y
void drawAngleLine(Mat& img, const Point2d& pt, const Circle& circle, double angle, int refAxis, Scalar colour) {
    // Draw line from point to center
    cv::line(img, pt, circle.center, colour, 2, cv::LINE_AA);

    // Draw the point
    cv::circle(img, pt, 6, colour, -1, cv::LINE_AA);

    // Calculate reference line direction based on axis
    // In image coordinates: +X is right, +Y is down
    // We display angles with +Y as "up", so we negate Y for display
    // refAxis: 0=+X (0°), 1=+Y (90° CCW), 2=-X (180°), 3=-Y (270°)
    double refAngleRad = refAxis * PI / 2.0;  // 0, 90, 180, 270 degrees in standard math coords
    Point2d refEnd(
        circle.center.x + circle.radius * std::cos(refAngleRad),
        circle.center.y - circle.radius * std::sin(refAngleRad)  // Negate to convert math Y to image Y
    );
    cv::line(img, circle.center, refEnd, colour, 2, cv::LINE_AA);

    // Draw axis label at the end of reference line
    cv::putText(img, AXIS_NAMES[refAxis], refEnd + Point2d(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv::LINE_AA);

    // Draw arc showing the angle - overlay on the circle to highlight the measured segment
    // OpenCV ellipse uses clockwise angles from the positive X-axis
    double startAngle = -refAxis * 90.0;  // Start from reference axis
    double endAngle = startAngle - angle;  // End at the measured angle (CCW is negative in OpenCV)
    int arcRadius = static_cast<int>(circle.radius);  // Same radius as the circle
    cv::ellipse(img, circle.center, cv::Size(arcRadius, arcRadius), 0, startAngle, endAngle, colour, 2, cv::LINE_AA);

    // Draw angle text
    char angleText[64];
    snprintf(angleText, sizeof(angleText), "%.1f deg", angle);
    cv::putText(img, angleText, pt + Point2d(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.7, colour, 2, cv::LINE_AA);
}

/// Display text with shadow
void shadowText(Mat& img, const string& text, int x, int y, Scalar colour) {
    cv::putText(img, text, cv::Point(x + 1, y + 1), cv::FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 2, cv::LINE_AA);
    cv::putText(img, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, colour, 2, cv::LINE_AA);
}

/// Load source frame from image or video file
/// Adapted from FicTrac's CVSource class
/// Returns true if successful, frame contains the loaded image
bool loadSource(const string& input, Mat& frame, string& sourceType) {
    // First try loading as video file
    try {
        std::shared_ptr<cv::VideoCapture> cap = std::make_shared<cv::VideoCapture>(input);
        if (cap->isOpened()) {
            Mat testFrame;
            *cap >> testFrame;
            if (!testFrame.empty()) {
                // Successfully read a frame from video
                testFrame.copyTo(frame);
                sourceType = "video";
                std::cout << "Loaded video file: " << input << std::endl;
                std::cout << "Using first frame (" << frame.cols << "x" << frame.rows << ")" << std::endl;
                return true;
            }
        }
    } catch (...) {
        // Video loading failed, try image
    }

    // Then try loading as image file
    try {
        frame = cv::imread(input);
        if (!frame.empty()) {
            sourceType = "image";
            std::cout << "Loaded image file: " << input << std::endl;
            std::cout << "Image size: " << frame.cols << "x" << frame.rows << std::endl;
            return true;
        }
    } catch (...) {
        // Image loading failed
    }

    return false;
}

void printUsage(const char* progName) {
    std::cout << "Usage: " << progName << " <image_or_video_path>" << std::endl;
    std::cout << std::endl;
    std::cout << "Interactive tool for measuring angles relative to a circle's center." << std::endl;
    std::cout << std::endl;
    std::cout << "Supported input formats:" << std::endl;
    std::cout << "  - Images: jpg, png, tiff, bmp, etc." << std::endl;
    std::cout << "  - Videos: mp4, avi, mov, etc. (uses first frame)" << std::endl;
    std::cout << std::endl;
    std::cout << "Instructions:" << std::endl;
    std::cout << "  1. Left-click to add points on the circle's edge (minimum 3 points)" << std::endl;
    std::cout << "  2. Right-click to remove the last point" << std::endl;
    std::cout << "  3. Press ENTER to confirm the circle fit" << std::endl;
    std::cout << "  4. Left-click to mark a point and see its angle to the center" << std::endl;
    std::cout << "  5. Press 'a' to cycle reference axis (+X -> +Y -> -X -> -Y)" << std::endl;
    std::cout << "  6. Press 'r' to restart and define a new circle" << std::endl;
    std::cout << "  7. Press ESC or 'q' to quit" << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    string inputPath = argv[1];
    Mat image;
    string sourceType;

    if (!loadSource(inputPath, image, sourceType)) {
        std::cerr << "Error: Could not load source: " << inputPath << std::endl;
        std::cerr << "Supported formats: images (jpg, png, tiff, bmp) and videos (mp4, avi, mov)" << std::endl;
        return 1;
    }

    // Initialize input data
    InputData inputData;
    inputData.newEvent = true;
    inputData.cursorPt = Point2d(-1, -1);
    inputData.anglePtSet = false;
    inputData.mode = MODE_CIRCLE_PTS;
    inputData.ptScl = 1.0f;

    // Create windows
    string windowName = "Elevation Measurement";
    string zoomWindowName = "Zoom";
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(zoomWindowName, cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback(windowName, onMouseEvent, &inputData);

    // Prepare zoom frame (initialize with zeros to avoid garbage display)
    int scaled_zoom_dim = static_cast<int>(ZOOM_DIM * ZOOM_SCL);
    Mat zoomFrame = Mat::zeros(ZOOM_DIM, ZOOM_DIM, image.type());

    Circle fittedCircle;
    fittedCircle.valid = false;

    // Reference axis for angle measurement (0 = +X, 1 = +Y, 2 = -X, 3 = -Y)
    int refAxis = AXIS_POS_X;

    std::cout << "\n=== Elevation Measurement Tool ===" << std::endl;
    std::cout << "Mode: CIRCLE ANNOTATION" << std::endl;
    std::cout << "Left-click to add points on the circle edge." << std::endl;
    std::cout << "Right-click to remove last point." << std::endl;
    std::cout << "Press ENTER when done, ESC to quit.\n" << std::endl;

    bool running = true;
    while (running) {
        // Create display frame
        Mat dispFrame = image.clone();

        // Draw clicked circle points
        int click_rad = std::max(static_cast<int>(dispFrame.cols / 250.0 + 0.5), 3);
        for (const auto& pt : inputData.circPts) {
            cv::circle(dispFrame, pt, click_rad, Scalar(0, 255, 255), 2, cv::LINE_AA);
        }

        // Fit and draw circle if we have enough points
        if (inputData.circPts.size() >= 3) {
            fitCircle2D(inputData.circPts, fittedCircle);
            if (fittedCircle.valid) {
                Scalar circleColor = (inputData.mode == MODE_CIRCLE_DONE || inputData.mode == MODE_ANGLE_PT)
                    ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
                drawCircle2D(dispFrame, fittedCircle, circleColor);
            }
        }

        // Draw angle measurement if in that mode
        if (inputData.mode == MODE_ANGLE_PT && inputData.anglePtSet && fittedCircle.valid) {
            double angle = calculateAngle(inputData.anglePt, fittedCircle, refAxis);
            drawAngleLine(dispFrame, inputData.anglePt, fittedCircle, angle, refAxis, Scalar(255, 0, 255));
        }

        // Draw cursor
        if (inputData.cursorPt.x >= 0 && inputData.cursorPt.y >= 0) {
            drawCursor(dispFrame, inputData.cursorPt, Scalar(0, 255, 0));
        }

        // Create zoom view (before adding text overlays)
        if (inputData.cursorPt.x >= 0 && inputData.cursorPt.y >= 0) {
            createZoomROI(zoomFrame, dispFrame, inputData.cursorPt, scaled_zoom_dim);
        }

        // Draw status text
        string statusText;
        switch (inputData.mode) {
            case MODE_CIRCLE_PTS:
                statusText = "Circle Points: " + std::to_string(inputData.circPts.size()) +
                            " (need 3+) | Left-click: add | Right-click: remove | ENTER: confirm";
                break;
            case MODE_CIRCLE_DONE:
            case MODE_ANGLE_PT:
                statusText = "CIRCLE CONFIRMED | Left-click: measure | 'a': change axis (" +
                            string(AXIS_NAMES[refAxis]) + ") | 'r': restart | ESC: quit";
                break;
        }
        shadowText(dispFrame, statusText, 10, 25, Scalar(255, 255, 255));

        // Show angle if set
        if (inputData.mode == MODE_ANGLE_PT && inputData.anglePtSet && fittedCircle.valid) {
            double angle = calculateAngle(inputData.anglePt, fittedCircle, refAxis);
            char angleStr[128];
            snprintf(angleStr, sizeof(angleStr), "Angle: %.2f degrees (from %s axis, CCW positive)", angle, AXIS_NAMES[refAxis]);
            shadowText(dispFrame, angleStr, 10, 55, Scalar(255, 0, 255));

            // Also print to console
            static double lastPrintedAngle = -9999;
            static int lastPrintedAxis = -1;
            if (std::abs(angle - lastPrintedAngle) > 0.01 || refAxis != lastPrintedAxis) {
                std::cout << "Angle: " << angle << " degrees (from " << AXIS_NAMES[refAxis] << ")" << std::endl;
                lastPrintedAngle = angle;
                lastPrintedAxis = refAxis;
            }
        }

        // Show circle info
        if (fittedCircle.valid) {
            char circleInfo[128];
            snprintf(circleInfo, sizeof(circleInfo), "Circle: center=(%.1f, %.1f) radius=%.1f",
                    fittedCircle.center.x, fittedCircle.center.y, fittedCircle.radius);
            shadowText(dispFrame, circleInfo, 10, dispFrame.rows - 15, Scalar(200, 200, 200));
        }

        // Display
        cv::imshow(windowName, dispFrame);
        cv::imshow(zoomWindowName, zoomFrame);

        // Handle keyboard input
        int key = cv::waitKey(30);

        // Check if window was closed by clicking X button (must be after imshow and waitKey)
        // WND_PROP_AUTOSIZE returns -1 if window doesn't exist
        if (cv::getWindowProperty(windowName, cv::WND_PROP_AUTOSIZE) == -1) {
            running = false;
        }
        if (key == 27 || key == 'q' || key == 'Q') {  // ESC or q
            running = false;
        } else if (key == 13 || key == 10) {  // ENTER
            if (inputData.mode == MODE_CIRCLE_PTS && fittedCircle.valid) {
                inputData.mode = MODE_ANGLE_PT;
                std::cout << "\n=== Circle Confirmed ===" << std::endl;
                std::cout << "Center: (" << fittedCircle.center.x << ", " << fittedCircle.center.y << ")" << std::endl;
                std::cout << "Radius: " << fittedCircle.radius << std::endl;
                std::cout << "\nMode: ANGLE MEASUREMENT" << std::endl;
                std::cout << "Left-click to measure angle from a point to the center." << std::endl;
                std::cout << "Press 'a' to cycle reference axis (+X -> +Y -> -X -> -Y)." << std::endl;
                std::cout << "Press 'r' to restart, ESC to quit.\n" << std::endl;
            }
        } else if (key == 'r' || key == 'R') {  // Reset
            inputData.circPts.clear();
            inputData.anglePtSet = false;
            inputData.mode = MODE_CIRCLE_PTS;
            fittedCircle.valid = false;
            refAxis = AXIS_POS_X;
            std::cout << "\n=== Reset ===" << std::endl;
            std::cout << "Mode: CIRCLE ANNOTATION" << std::endl;
            std::cout << "Left-click to add points on the circle edge.\n" << std::endl;
        } else if (key == 'a' || key == 'A') {  // Cycle reference axis
            if (inputData.mode == MODE_ANGLE_PT || inputData.mode == MODE_CIRCLE_DONE) {
                refAxis = (refAxis + 1) % 4;  // Cycle: +X -> +Y -> -X -> -Y -> +X
                std::cout << "Reference axis changed to: " << AXIS_NAMES[refAxis] << std::endl;
            }
        }

        inputData.newEvent = false;
    }

    cv::destroyAllWindows();
    return 0;
}

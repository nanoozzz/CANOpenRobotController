/**
 * \file FittsCore.h
 * \brief Hardware-independent core of the M2FittsMachine CORC application.
 *
 * Provides (i) trial-table loading from CSV, (ii) the PD control law, (iii) a rate-limited
 * set-point generator used only to bring the robot home, (iv) per-trial event detection
 * (kinematic onset, target entries, dwell completion, time-out) and (v) the per-trial
 * results writer. It depends only on the C++17 standard library and Eigen, so it can be
 * compiled and exercised offline (tools/M2FittsSim) without CORC or the robot.
 *
 * Provenance: drafted with AI assistance (Claude, Anthropic), September 2026, for a
 * robot-guidance Fitts' task on the ArmMotus M2 using CORC. Review and validate before use.
 * Licence: Apache-2.0 (same as CORC).
 */
#ifndef FITTS_CORE_H
#define FITTS_CORE_H

#include <Eigen/Dense>

#include <cstdint>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace fitts {

typedef Eigen::Vector2d Vec2;
const double kNaN = std::numeric_limits<double>::quiet_NaN();

/** One row of the trial table (values as read from the file, in file units, cm by default). */
struct Trial {
    int index = 0;     //!< 1-based trial number (row order in the file)
    int csv_line = 0;  //!< Line number in the CSV file (traceability)
    double D = 0.;     //!< Displacement (movement amplitude), file units
    double W = 0.;     //!< Target width, file units
};

/** PD gains and limits in end-effector space, one value per axis [x, y]. */
struct PDGains {
    Vec2 kp = Vec2(400., 400.);  //!< Proportional gain [N/m]
    Vec2 kd = Vec2(80., 80.);    //!< Derivative gain [N.s/m]
    double f_max = 30.;          //!< Per-axis force saturation [N]
    double vel_filter_hz = 0.;   //!< 1st-order low-pass cut-off on measured velocity [Hz]; <= 0 disables it
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** Complete application configuration. In-code defaults are placeholders: the YAML file is authoritative. */
struct FittsConfig {
    // Trial file
    std::string trials_file = "/home/nle/Fitts/CANOpenRobotController/schedule/group_1.csv";  //!< Relative to CORC config/ folder, or absolute path
    std::string d_column = "B";                     //!< Spreadsheet letter of the D column
    std::string w_column = "C";                     //!< Spreadsheet letter of the W column
    double units_to_m = 0.01;                       //!< Multiply file values by this to get metres (cm -> m)
    int expected_trials = 180;                      //!< Abort if the file holds another number of trials (0 = no check)
    int start_trial = 1;                            //!< 1-based index of first trial to run (resume a block)
    // Task
    Vec2 home = Vec2(0.40, 0.20);                   //!< Home position [m]; y is held here throughout
    int direction = -1;                             //!< -1: targets at x_home - D ; +1: x_home + D
    double tolerance_factor = 0.5;                  //!< Success band = |x - x_target| <= tolerance_factor * W
    double dwell_time = 1.0;                        //!< Continuous time inside the band required for success [s]
    double acquisition_timeout = 5.0;               //!< Trial fails if outside the band at/after this time [s]
    double onset_speed = 0.01;                      //!< |dx| threshold defining kinematic onset [m/s]
    // Home settling
    double home_tolerance = 0.005;                  //!< |x - home| considered "at home" [m]
    double rest_speed = 0.005;                      //!< |v| considered "at rest" [m/s]
    double rest_time = 0.5;                         //!< Continuous rest at home before the next trial [s]
    double settle_timeout = 10.0;                   //!< Extra time allowed to settle at home before pausing [s]
    double return_ref_speed = 0.10;                 //!< Reference speed when returning home [m/s] (0 = step)
    double homing_ref_speed = 0.05;                 //!< Reference speed from calibration corner to home [m/s]
    // Controller
    PDGains pd;
    bool friction_compensation = false;             //!< Add RobotM2 friction feedforward to the PD force
    // Safety and workspace
    double max_speed = 0.8;                         //!< App-level speed limit -> FaultState [m/s]
    Vec2 workspace_x = Vec2(0.0, 0.625);            //!< Joint range of x as defined in RobotM2 (CORC) [m]
    Vec2 workspace_y = Vec2(0.0, 0.440);            //!< Joint range of y as defined in RobotM2 (CORC) [m]
    double workspace_tolerance = 0.01;              //!< Allowed excursion beyond the range before fault [m]
    double target_margin = 0.02;                    //!< Min. distance between target band and range ends [m]
    double brake_damping = 40.0;                    //!< Damping used by FaultState to stop the robot [N.s/m]
    // Calibration against the lower mechanical stops
    double calib_force = 20.0;                      //!< [N]
    double calib_damping = 3.0;                     //!< [N.s/m]
    double calib_still_speed = 0.005;               //!< [m/s]
    double calib_still_time = 1.0;                  //!< [s]
    // Output
    std::string log_folder = "logs";                //!< Relative to the working directory of the app

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** Information about the loaded CSV file (for provenance). */
struct CsvInfo {
    std::string path;
    std::string header;         //!< Header row as found in the file ("" if none)
    char delimiter = ',';
    std::uint64_t fnv1a64 = 0;  //!< FNV-1a 64-bit fingerprint of the raw bytes (identifies the file, not cryptographic)
};

/** "A" -> 0, "B" -> 1, "W" -> 22, "AA" -> 26 (case-insensitive). Returns -1 if invalid. */
int columnLetterToIndex(const std::string &letters);

/**
 * Load the trial table. Handles an optional header row, UTF-8 BOM, CRLF line endings, blank rows,
 * quoted fields, and ';' or TAB separated files with decimal commas. Throws std::runtime_error with
 * the file line number on any problem (nothing is guessed silently).
 */
std::vector<Trial> loadTrialsCsv(const std::string &path, int d_col, int w_col, int expected_rows, CsvInfo *info = nullptr);

double targetX(const Trial &t, const FittsConfig &cfg);          //!< x_home + direction * D [m]
double targetTolerance(const Trial &t, const FittsConfig &cfg);  //!< tolerance_factor * W [m]

std::vector<std::string> validateConfig(const FittsConfig &cfg);                                 //!< Empty if valid
std::vector<std::string> validateTrials(const std::vector<Trial> &trials, const FittsConfig &cfg);  //!< Empty if valid

std::string describeConfig(const FittsConfig &cfg);             //!< Human-readable "key: value" dump
std::string localTimestamp(const char *strftime_format, bool with_milliseconds);

/**
 * PD law in end-effector space (per axis):
 *     F = Kp (x_ref - x) + Kd (v_ref - v_f),   then |F_i| <= f_max
 * With a fixed set-point (v_ref = 0) the derivative acts on the measured velocity only,
 * so a step change of the set-point produces no derivative "kick".
 */
class PDController {
   public:
    PDController() = default;
    explicit PDController(const PDGains &g) : g_(g) {}
    void setGains(const PDGains &g) { g_ = g; }
    const PDGains &gains() const { return g_; }
    void reset() { initialised_ = false; }
    Vec2 compute(const Vec2 &x_ref, const Vec2 &v_ref, const Vec2 &x, const Vec2 &v, double dt, bool *saturated = nullptr);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   private:
    PDGains g_;
    Vec2 v_filt_ = Vec2::Zero();
    bool initialised_ = false;
};

/** Straight-line reference moving towards a goal at constant speed (max_speed <= 0: jumps to goal). */
class RateLimitedReference {
   public:
    void reset(const Vec2 &start, const Vec2 &goal, double max_speed);
    Vec2 update(double dt, Vec2 &v_ref);
    bool done() const { return done_; }
    double remainingDistance() const { return (goal_ - ref_).norm(); }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   private:
    Vec2 ref_ = Vec2::Zero();
    Vec2 goal_ = Vec2::Zero();
    double vmax_ = 0.;
    bool done_ = true;
};

enum class TrialOutcome { Running, Success, Timeout, Aborted };
const char *outcomeName(TrialOutcome o);

/** Everything recorded for one trial. All times are relative to trial (command) onset, in seconds. */
struct TrialResult {
    Trial trial;
    std::string wall_clock;                 //!< Local date-time at trial onset
    double x_home = kNaN, y_home = kNaN;    //!< [m]
    double x_target = kNaN;                 //!< [m]
    double tolerance = kNaN;                //!< tolerance_factor * W [m]
    double x_start = kNaN;                  //!< Measured x at onset [m]
    double D_effective = kNaN;              //!< direction * (x_target - x_start) [m]
    TrialOutcome outcome = TrialOutcome::Running;
    double rt_kinematic = kNaN;             //!< First sample with |dx| >= onset_speed
    double mt_first_entry = kNaN;           //!< First entry into the band
    double mt_final_entry = kNaN;           //!< Entry that started the completed dwell (success only)
    double mt_first_entry_kin = kNaN;       //!< mt_first_entry - rt_kinematic
    double mt_final_entry_kin = kNaN;       //!< mt_final_entry - rt_kinematic
    double task_time = kNaN;                //!< Dwell completion (success) or trial end (time-out/abort)
    int n_entries = 0;                      //!< Number of entries into the band
    double peak_speed_x = 0.;               //!< [m/s]
    double t_peak_speed_x = kNaN;
    double overshoot = 0.;                  //!< max(direction * (x - x_target), 0) [m]
    double endpoint_error = kNaN;           //!< direction * (x - x_target) at selection or trial end [m]
    double y_max_dev = 0.;                  //!< max |y - y_home| [m]
    double sat_fraction = 0.;               //!< Fraction of control samples with PD force saturation
    double dt_mean = kNaN, dt_max = kNaN;   //!< Control-period statistics [s]
    unsigned long samples = 0;
    std::string note;
};

/**
 * Detects trial events sample by sample. Rules:
 *  - in target  <=> |x - x_target| <= tolerance_factor * W (movement axis x only)
 *  - success    <=> in target continuously for dwell_time (leaving the band resets the dwell)
 *  - time-out   <=> outside the band at t >= acquisition_timeout (a dwell already under way may finish)
 */
class TrialMonitor {
   public:
    void start(const Trial &trial, const FittsConfig &cfg, const Vec2 &x_start, const std::string &wall_clock);
    bool update(double t, const Vec2 &x, const Vec2 &v, bool saturated);  //!< Returns true once finished
    void abort(double t, const std::string &reason);
    bool finished() const { return result_.outcome != TrialOutcome::Running; }
    bool inTarget() const { return in_target_; }
    const TrialResult &result() const { return result_; }

   private:
    void finalise();
    TrialResult result_;
    double dwell_time_ = 1., timeout_ = 5., onset_speed_ = 0.01;
    int direction_ = -1;
    bool in_target_ = false;
    double entry_time_ = kNaN, t_prev_ = kNaN, dt_sum_ = 0.;
    unsigned long sat_samples_ = 0;
};

/** Per-trial results CSV (one row per trial, flushed immediately). */
class ResultsWriter {
   public:
    bool open(const std::string &path);
    bool write(const TrialResult &r);
    void close();
    bool isOpen() const { return f_.is_open(); }
    const std::string &path() const { return path_; }
    static std::string header();

   private:
    std::ofstream f_;
    std::string path_;
};

}  // namespace fitts

#endif  // FITTS_CORE_H

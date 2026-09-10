#ifndef TRIALTABLE_H
#define TRIALTABLE_H

#include <string>
#include <vector>

/**
 * \brief One experimental trial as read from the CSV.
 *
 * A and W are stored in METRES (the raw CSV values multiplied by UNIT_SCALE).
 * IDfile is the value in your CSV; IDfitts and IDshannon are recomputed here so
 * the formulation can be verified rather than assumed.
 */
struct Trial {
    int    index      = 0;
    double A          = 0.0;  //!< amplitude, m
    double W          = 0.0;  //!< target width, m
    double IDfile     = 0.0;  //!< ID as given in the CSV
    double IDfitts    = 0.0;  //!< log2(2A/W)   (Fitts, 1954)
    double IDshannon  = 0.0;  //!< log2(A/W+1)  (MacKenzie, 1992)
};

/**
 * \brief Loads, validates and holds the trial list.
 */
class TrialTable {
   public:
    /**
     * \brief Load and validate the trial CSV.
     * \param path        CSV path, columns index,A,W,ID (header row optional)
     * \param unitScale   multiplier converting CSV A/W into metres
     * \param minTarget   lowest admissible target coordinate on the task axis, m
     * \param maxTarget   highest admissible target coordinate on the task axis, m
     * \param home        home coordinate on the task axis, m
     * \param direction   +1 or -1
     * \return true only if every row parsed AND every target is reachable
     */
    bool load(const std::string &path, double unitScale, double minTarget,
              double maxTarget, double home, double direction);

    size_t size() const { return trials_.size(); }
    const Trial &at(size_t i) const { return trials_.at(i); }

    //! Which ID formulation the CSV column matches ("Fitts", "Shannon", or "neither").
    const std::string &formulation() const { return formulation_; }

   private:
    std::vector<Trial> trials_;
    std::string formulation_ = "unknown";
};

#endif  // TRIALTABLE_H
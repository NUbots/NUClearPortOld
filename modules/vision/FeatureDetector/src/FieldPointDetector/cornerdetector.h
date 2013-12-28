#ifndef CORNERDETECTOR_H
#define CORNERDETECTOR_H

#include "Vision/VisionTypes/VisionFieldObjects/cornerpoint.h"
#include "Vision/VisionTypes/VisionFieldObjects/fieldline.h"

class CornerDetector
{
public:
    CornerDetector(double tolerance);

    std::vector<CornerPoint> run(const std::vector<FieldLine>& lines) const;

    void setTolerance(double tolerance);

private:
    CornerPoint::TYPE findCorner(Vector2<NUPoint> ep1, Vector2<NUPoint> ep2, NUPoint intersection, double tolerance) const;

    double m_tolerance;
};

#endif // CORNERDETECTOR_H

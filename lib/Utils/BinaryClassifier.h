

#pragma once

#include <vector>

namespace AprilTags {


class BinaryClassifier {
public:

	BinaryClassifier( float whiteMean = 1.0 );

	int classify( double v ) const;

	void startTraining( void );
	void trainingPoint( double v );
	void endTraining( void );

protected:

	double mean[2];
	double invSqrtVar[2];
	double invVar[2];
	std::vector< double > _trainPoints[2];

};

}



#include "BinaryClassifier.h"

#include <math.h>

namespace AprilTags {

// TODO:  Add unit tests for BinaryClassifier

BinaryClassifier::BinaryClassifier( float whiteMean )
{
	mean[0] = 0;
	mean[1] = whiteMean;

	float stddev = (mean[1]-mean[0]) / 3.0;
	invVar[0] = invVar[1] = 1.0 / (stddev*stddev);
	invSqrtVar[0] = invSqrtVar[1] = 1.0 / (stddev*sqrt( 2 * M_PI  ));
}

int BinaryClassifier::classify( double v ) const
{
	double p0 = invSqrtVar[0] * exp( -0.5f * (v-mean[0]) * (v-mean[0]) * invVar[0]),
				 p1 = invSqrtVar[1] * exp( -0.5f * (v-mean[1]) * (v-mean[1]) * invVar[1]);

	return ( p1 > p0 ) ? 1 : 0;
}

void BinaryClassifier::startTraining( void )
{
	_trainPoints[0].clear();
	_trainPoints[1].clear();
}

void BinaryClassifier::trainingPoint( double v )
{
	int which = classify( v );
	_trainPoints[which].push_back( v );
}

void BinaryClassifier::endTraining( void )
{
	for( int i = 0; i < 2; ++i ) {
		mean[i] = 0.0;
		float var = 0;

		if( _trainPoints[i].size() < 2 ) continue;

		for( float f : _trainPoints[i])
			mean[i] += f;

		mean[i] /= _trainPoints[i].size();

		for( float f : _trainPoints[i])
			var += (f-mean[i])*(f-mean[i]);

		var /= (_trainPoints[i].size()-1);

		invVar[i] = 1.0/var;
		invSqrtVar[i] = 1.0 / sqrt( 2 * M_PI * var );
	}
}



}

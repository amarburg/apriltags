#pragma once

#include <memory>

#include "CornerDetector/CornerDetectorTypes.h"
#include "AprilTags/CornerArray.h"
#include "Utils/MathUtil.h"

namespace AprilTags {

class Hypothesis;
class SecondPoint;
class InitialHypothesis;

class Point {
public:
	Point( const std::shared_ptr< Intersection > &p )
		: intersection( p )
	{;}

	virtual ~Point() {;}

	virtual Hypothesis &addHypothesis( const CornerArray::ArrayElement &corner, unsigned int spin );

	shared_ptr< Intersection > intersection;
	std::vector< Hypothesis > hypotheses;
};

class Hypothesis {
public:
	Hypothesis( const Point &point_,
							const CornerArray::ArrayElement &corner_,
							unsigned int spin_)
		: point( point_ ),
			corner( corner_ ),
 			spin( spin_ )
	{;}

	// (angle in image space + theta) = angle in CornerArray space
	float theta( void ) const
		{ return M_PI/2.0 * spin - corner.rotation - point.intersection->basis[0]; }

	// Basis of origin rotated into CornerArray space
	float basisInCorner( void ) const
		{ //return origin->basis[0] + originTheta();
			return M_PI/2.0 * spin - corner.rotation; }

	float error( const cv::Point2f &pred, float imgDeltaBasis ) const
		{
			cv::Point2f posError = pred - corner.center;
			float deltaError = MathUtil::angleError( imgDeltaBasis, basisInCorner() );

			return (posError.x*posError.x) + (posError.y*posError.y)
						+ 1e6*deltaError*deltaError;
		}

	const Point &point;
	const CornerArray::ArrayElement &corner;
	unsigned int spin;
};


class OriginPoint : public Point {
public:
	OriginPoint( const std::shared_ptr< Intersection > &p )
		: Point( p )
	{ ; }

	Point &addSecondPoint( const std::shared_ptr< Intersection > &p );

	void evaluateInitialHypotheses( void );
	unsigned int keepInitialHypotheses( unsigned int c );
	void reevaluateHypotheses( unsigned int minEvalPoints  );

	std::vector< SecondPoint > seconds;
	multimap< float, InitialHypothesis > eval;
};

class SecondPoint : public Point {
public:
	SecondPoint( const OriginPoint &origin_, const std::shared_ptr< Intersection > &p )
		: Point( p ),
			origin( origin_ )
	{ ; }

	virtual ~SecondPoint() {;}

	float imgBearing( void ) const
		{ return  atan2( intersection->center.y - origin.intersection->center.y,
											intersection->center.x - origin.intersection->center.x ); }

	float imgDeltaBasis( void ) const
		{ return MathUtil::angleError( origin.intersection->basis[0], intersection->basis[0]); }

	// float deltaBasis( void ) const
	// 	{ return MathUtil::angleError( originPoint().intersection()->basis[0], _point->basis[0]); }

	const OriginPoint &origin;
};

class EvalPoint : public Point {
public:
	EvalPoint( const shared_ptr<Intersection> &p, const cv::Point2f &pred, float imgDeltaBasis )
		: Point(p), cornerPred( pred ), deltaBasisPred( imgDeltaBasis )
	{;}

	virtual Hypothesis &addHypothesis( const CornerArray::ArrayElement &corner, unsigned int spin );

	void keepBestHypothesis( void );
	float lowestError( void ) const;
	// const Hypothesis &bestHypothesis( void ) const { return scored.begin()->second; }

	// NOTE:  Trying copy on return as having problems with return of reference
	const Hypothesis &hypothesis( unsigned int c ) const;
	cv::Point2f cornerPred;
	float deltaBasisPred;
	multimap< float, int > scored;

};


class InitialHypothesis {
public:
	InitialHypothesis( const Hypothesis &originHyp_, const Hypothesis &secondHyp_ )
		: originHyp( originHyp_ ), secondHyp( secondHyp_ ),
			origin( originHyp_.point ), second( dynamic_cast< const SecondPoint &>(secondHyp_.point) )
	{
		sim = MathUtil::solveSimilarity( origin.intersection->center, second.intersection->center,
																		originHyp.corner.center, secondHyp.corner.center );
	}

	// Bearing from origin to second projected into CornerArray space
	float imgBearingInCorner( void ) const
		{ return second.imgBearing() + originHyp.theta(); }

	float cornerBearing() const
		{ return atan2( secondHyp.corner.center.y - originHyp.corner.center.y,
										secondHyp.corner.center.x - originHyp.corner.center.x ); }

	// Difference between two above bearings
	float bearingError( void ) const
		{ return MathUtil::angleError(cornerBearing(), imgBearingInCorner()); }

	float cornerDeltaBasis( void ) const
		{ return MathUtil::angleError( originHyp.basisInCorner(), secondHyp.basisInCorner()); }

	float spinError( void ) const
		{ return MathUtil::angleError( cornerDeltaBasis(), second.imgDeltaBasis() ); }

	float initialError( void ) const
		{ return spinError()*spinError() + 1e2*bearingError()*bearingError(); }

	// Handle evaluation of points

	EvalPoint &addPoint( const shared_ptr< Intersection > &p );
	void abortPoint( );
	cv::Point2f project( const Point &p );

	float totalError( void ) const;

	const Hypothesis &originHyp, &secondHyp;
	const Point &origin;
 	const SecondPoint &second;
	cv::Matx23f sim;

 std::vector< EvalPoint > evalPoints;
};

// Hypothesis about location of origin
// class OriginHypothesis {
// public:
// 	OriginHypothesis( const OriginPoint &parent,
// 			const CornerArray::ArrayElement &corner, unsigned int spin )
// 		: _parent( parent ), _corner( corner ), _spin( spin )
// 	{;}
//
// 	// (angle in image space + originTheta) = angle in CornerArray space
// 	float theta( void ) const
// 		{ return M_PI/2.0 * _spin - _corner.rotation - _parent.intersection()->basis[0]; }
//
// 	// Basis of origin rotated into CornerArray space
// 	float basisInArray( void ) const
// 		{ //return origin->basis[0] + originTheta();
// 			return M_PI/2.0 * _spin - _corner.rotation; }
//
// 	// // Difference between two above bearings
// 	// float bearingError( void ) const
// 	// 	{ return MathUtil::angleError(bearingArray(), _parent.ImageInArray()); }
// 	//
// 	// float totalError( void ) const { return 0; }
//
// 	const CornerArray::ArrayElement &corner( void ) const { return _corner; }
//
// 	//const std::shared_ptr< Intersection > &point( void ) const { return _parent.origin(); }
//
// 	std::unique_ptr< class SecondPoint > &addSecondPoint( const shared_ptr< Intersection > &second );
//
//
//
//
// 	const OriginPoint &_parent;
// 	const CornerArray::ArrayElement &_corner;
// 	unsigned int _spin;
//
// 	std::vector< std::unique_ptr< class SecondPoint > > _children;
// };

// class SecondPoint {
// public:
// 	SecondPoint( const OriginHypothesis &parent, const shared_ptr< Intersection > &second )
// 		: _parent( parent ),
// 			_point( second )
// 	{;}
//
// 	// Bearing from origin to second in image space
// 	 float bearing( void ) const
// 	 	{ return  atan2( _point->center.y - originPoint().intersection()->center.y,
// 										 _point->center.x - originPoint().intersection()->center.x ); }
//
// 	// Bearing from origin to second projected into CornerArray space
// 	float bearingInArray( void ) const
// 		{ return bearing() + _parent.theta(); }
//
// 	float deltaBasis( void ) const
// 		{ return MathUtil::angleError( originPoint().intersection()->basis[0], _point->basis[0]); }
//
// 	std::unique_ptr< class SecondHypothesis > &addSecondHypothesis( const CornerArray::ArrayElement &corner, unsigned int spin );
//
// 	const OriginHypothesis &originHypothesis() const { return _parent; }
// 	const OriginPoint &originPoint() const { return _parent._parent; }
//
// 	const std::multimap< float, std::unique_ptr< SecondHypothesis > > &hypotheses( void ) const { return _children; }
//
// 	const shared_ptr< Intersection > &intersection( void ) const { return _point; }
//
// 	const OriginHypothesis &_parent;
// 	shared_ptr< Intersection > _point;
//
// 	std::multimap< float, std::unique_ptr< SecondHypothesis > > _children;
// };
//
// // Hypothesis about location of second
// class SecondHypothesis {
// public:
// 	SecondHypothesis( const SecondPoint &parent, const CornerArray::ArrayElement &corner, unsigned int spin )
// 		: _parent( parent ), _corner( corner ), _spin( spin ),
// 		sim( MathUtil::solveSimilarity( _parent.originPoint().intersection()->center, _parent.intersection()->center,
// 																			_parent.originHypothesis().corner().center, _corner.center ))
// {
// 	;
// }
//
// 	// given a spin match, the angle in image space + secondTheta = angle in CornerArray space
// 	// float theta( void ) const
// 	// 	{ return M_PI/2.0 * _spin - _corner.rotation - _parent.second->basis[0]; }
//
// 	// Projects second basis into CornerArray space
// 	float basisInArray( void ) const
// 		{ return M_PI/2.0 * _spin - _corner.rotation; }
//
// 	// Bearing from the proposed matches for origin to second in CornerArray space
// 	float bearingArray( void ) const
// 		{ return atan2( _corner.center.y - _parent.originHypothesis().corner().center.y,
// 										_corner.center.x - _parent.originHypothesis().corner().center.x ); }
//
// 	// Differente between two above bearings
// 	float bearingError( void ) const
// 		{ return MathUtil::angleError(bearingArray(), _parent.bearingInArray()); }
//
// 	// Differenes of bases in image frame should be the same (very close to)
// 	// difference of bases in CornerArray frame
// 	// TODO.  Expand this out to it's final form
// 	float spinError( void ) const
// 		{ return MathUtil::angleError(MathUtil::angleError( _parent.originHypothesis().basisInArray(), basisInArray()),
// 																	_parent.deltaBasis() ); }
//
// 	// Sum of errors
// 	float error( void ) const
// 		{ return spinError()*spinError() + 1e2*bearingError()*bearingError(); }
//
// 	void addPoint( const shared_ptr<Intersection> intersection );
//
// 	const SecondPoint &_parent;
// 	const CornerArray::ArrayElement &_corner;
// 	unsigned int _spin;
// 	cv::Matx23f sim;
// };
//

}

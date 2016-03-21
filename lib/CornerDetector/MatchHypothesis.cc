#include <iostream>

#include "MatchHypothesis.h"

#include "Utils/range.hpp"


namespace AprilTags {

using namespace util::lang;

Hypothesis &Point::addHypothesis( const CornerArray::ArrayElement &corner, unsigned int spin )
{
	hypotheses.emplace_back( *this, corner, spin );
	return hypotheses.back();
}



Point &OriginPoint::addSecondPoint( const std::shared_ptr< Intersection > &p )
{
	seconds.emplace_back( *this, p );
	return seconds.back();
}

void OriginPoint::evaluateInitialHypotheses( void )
{
	std::cout << "Evaluating " << hypotheses.size() << " origin hypotheses with "
														<< seconds.size() << " second points and "
														<< seconds.front().hypotheses.size() << " second hypotheses" << std::endl;
	for( auto const &firstHyp : hypotheses ) {
		for( auto const &second : seconds ) {
			for( auto const &secondHyp : second.hypotheses ) {
				InitialHypothesis hyp( firstHyp, secondHyp );

				eval.emplace( hyp.initialError(), hyp);
			}
		}
	}
}

unsigned int OriginPoint::keepInitialHypotheses( unsigned int c )
{
	if( eval.size() > c ) {
		auto itr = eval.begin();
		for( unsigned int i=0 ; i < c ; ++i ) ++itr;

		eval.erase( itr, eval.end() );
	}
	return eval.size();
}

void OriginPoint::reevaluateHypotheses( unsigned int minEvalPoints )
{
	multimap< float, InitialHypothesis > reEval;

	for( auto const &hypPair : eval ) {
		const InitialHypothesis &hyp( hypPair.second );
		if( hyp.evalPoints.size() < minEvalPoints ) {
			// std::cout << "Hypothesis has only " << hyp.evalPoints.size() << "/" << minEvalPoints << " eval points, dropping" << std::endl;
			continue;
		}
		reEval.emplace( hyp.totalError(), hyp );
	}

	eval.swap( reEval );
}

EvalPoint &InitialHypothesis::addPoint( const shared_ptr< Intersection > &p )
{
	evalPoints.emplace_back( p, sim*p->center, second.imgDeltaBasis() ); //MathUtil::angleError( origin.intersection->basis[0], p->basis[0]) );
	return evalPoints.back();
}

void InitialHypothesis::abortPoint( void )
{
	evalPoints.pop_back();
}

float InitialHypothesis::totalError( void ) const
{
	// Mean error?
	float sumError = 0.0;

	for( auto const &pt : evalPoints ) {
		sumError += pt.lowestError();
	}

	return sumError / evalPoints.size();
}

Hypothesis &EvalPoint::addHypothesis( const CornerArray::ArrayElement &corner, unsigned int spin )
{
	Hypothesis &hyp( Point::addHypothesis( corner, spin ) );
	scored.emplace( hyp.error( cornerPred, deltaBasisPred ), hypotheses.size()-1 );
	return hyp;
}

void EvalPoint::keepBestHypothesis( void )
{
	// if( hypotheses.size() > 1 ) {
	// 	const std::pair< float, const Hypothesis & > &pair( *(scored.begin()) );
	// 	// Make a copy (best way to do it for now)
	//
	// 	std::vector< Hypothesis > newHyp;
	// 	newHyp.push_back( pair.second );
	// 	hypotheses.swap( newHyp );
	//
	// 	multimap< float, const Hypothesis & > newScored;
	// 	newScored.insert( pair );
	// 	scored.swap( newScored );
	//
	// 	CV_Assert( hypotheses.size() == 1 && scored.size() == 1 );
	// }
}

float EvalPoint::lowestError( void ) const
{
	if( hypotheses.size() == 0 )
		return 0.0;
	else
		return scored.begin()->first;
}

const Hypothesis &EvalPoint::hypothesis( unsigned int c ) const
{
	auto itr = scored.cbegin();
	for( unsigned int i = 0; i < std::min(c,(unsigned int)scored.size()-1) ; ++i ) ++itr;
	return hypotheses[itr->second];
}




// std::unique_ptr< class SecondPoint > &OriginHypothesis::addSecondPoint( const shared_ptr< Intersection > &second )
// {
// 	_children.emplace_back( new SecondPoint(*this, second ) );
// 	return _children.back();
// }
//
// std::unique_ptr< class SecondHypothesis > &SecondPoint::addSecondHypothesis( const CornerArray::ArrayElement &corner, unsigned int spin )
// {
// 	SecondHypothesis *hyp = new SecondHypothesis( *this, corner, spin );
// 	float error = hyp->error();
// 	return _children.emplace( std::make_pair( error, unique_ptr< class SecondHypothesis >(hyp) ))->second;
// }



}

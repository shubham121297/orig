/*
    open source routing machine
    Copyright (C) Dennis Luxen, others 2010

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU AFFERO General Public License as published by
the Free Software Foundation; either version 3 of the License, or
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
or see http://www.gnu.org/licenses/agpl.txt.
 */



#ifndef BASICROUTINGINTERFACE_H_
#define BASICROUTINGINTERFACE_H_

#include "../DataStructures/RawRouteData.h"
#include "../DataStructures/SearchEngineData.h"
#include "../Util/ContainerUtils.h"
#include "../Util/SimpleLogger.h"

#include <boost/assert.hpp>
#include <boost/noncopyable.hpp>

#include <climits>

#include <stack>

template<class DataFacadeT>
class BasicRoutingInterface : boost::noncopyable{
protected:
    DataFacadeT * facade;
public:
    BasicRoutingInterface(DataFacadeT * facade) : facade(facade) { }
    virtual ~BasicRoutingInterface(){ };

    inline void RoutingStep(
        SearchEngineData::QueryHeap & _forwardHeap,
        SearchEngineData::QueryHeap & _backwardHeap,
        NodeID *middle,
        int *_upperbound,
        const int edgeBasedOffset,
        const bool forwardDirection
    ) const {
        const NodeID node = _forwardHeap.DeleteMin();
        const int distance = _forwardHeap.GetKey(node);
        //SimpleLogger().Write() << "Settled (" << _forwardHeap.GetData( node ).parent << "," << node << ")=" << distance;
        if(_backwardHeap.WasInserted(node) ){
            const int newDistance = _backwardHeap.GetKey(node) + distance;
            if(newDistance < *_upperbound ){
                if(newDistance>=0 ) {
                    *middle = node;
                    *_upperbound = newDistance;
                } else {
                }
            }
        }

        if(distance-edgeBasedOffset > *_upperbound){
            _forwardHeap.DeleteAll();
            return;
        }

        //Stalling
        for(
            EdgeID edge = facade->BeginEdges( node );
            edge < facade->EndEdges(node);
            ++edge
         ) {
            const typename DataFacadeT::EdgeData & data = facade->GetEdgeData(edge);
            bool backwardDirectionFlag = (!forwardDirection) ? data.forward : data.backward;
            if(backwardDirectionFlag) {
                const NodeID to = facade->GetTarget(edge);
                const int edgeWeight = data.distance;

                BOOST_ASSERT_MSG( edgeWeight > 0, "edgeWeight invalid" );

                if(_forwardHeap.WasInserted( to )) {
                    if(_forwardHeap.GetKey( to ) + edgeWeight < distance) {
                        return;
                    }
                }
            }
        }

        for ( EdgeID edge = facade->BeginEdges( node ); edge < facade->EndEdges(node); ++edge ) {
            const typename DataFacadeT::EdgeData & data = facade->GetEdgeData(edge);
            bool forwardDirectionFlag = (forwardDirection ? data.forward : data.backward );
            if(forwardDirectionFlag) {

                const NodeID to = facade->GetTarget(edge);
                const int edgeWeight = data.distance;

                BOOST_ASSERT_MSG( edgeWeight > 0, "edgeWeight invalid" );
                const int toDistance = distance + edgeWeight;

                //New Node discovered -> Add to Heap + Node Info Storage
                if ( !_forwardHeap.WasInserted( to ) ) {
                    _forwardHeap.Insert( to, toDistance, node );
                }
                //Found a shorter Path -> Update distance
                else if ( toDistance < _forwardHeap.GetKey( to ) ) {
                    _forwardHeap.GetData( to ).parent = node;
                    _forwardHeap.DecreaseKey( to, toDistance );
                    //new parent
                }
            }
        }
    }

    inline void UnpackPath(const std::vector<NodeID> & packedPath, std::vector<_PathData> & unpackedPath) const {
        const unsigned sizeOfPackedPath = packedPath.size();
        std::stack<std::pair<NodeID, NodeID> > recursionStack;

        //We have to push the path in reverse order onto the stack because it's LIFO.
        for(unsigned i = sizeOfPackedPath-1; i > 0; --i){
            recursionStack.push(std::make_pair(packedPath[i-1], packedPath[i]));
        }

        std::pair<NodeID, NodeID> edge;
        while(!recursionStack.empty()) {
            edge = recursionStack.top();
            recursionStack.pop();

            EdgeID smallestEdge = SPECIAL_EDGEID;
            int smallestWeight = INT_MAX;
            for(EdgeID eit = facade->BeginEdges(edge.first);eit < facade->EndEdges(edge.first);++eit){
                const int weight = facade->GetEdgeData(eit).distance;
                if(facade->GetTarget(eit) == edge.second && weight < smallestWeight && facade->GetEdgeData(eit).forward){
                    smallestEdge = eit;
                    smallestWeight = weight;
                }
            }

            if(smallestEdge == SPECIAL_EDGEID){
                for(EdgeID eit = facade->BeginEdges(edge.second);eit < facade->EndEdges(edge.second);++eit){
                    const int weight = facade->GetEdgeData(eit).distance;
                    if(facade->GetTarget(eit) == edge.first && weight < smallestWeight && facade->GetEdgeData(eit).backward){
                        smallestEdge = eit;
                        smallestWeight = weight;
                    }
                }
            }
            BOOST_ASSERT_MSG(smallestWeight != INT_MAX, "edge id invalid");

            const typename DataFacadeT::EdgeData& ed = facade->GetEdgeData(smallestEdge);
            if(ed.shortcut) {//unpack
                const NodeID middle = ed.id;
                //again, we need to this in reversed order
                recursionStack.push(std::make_pair(middle, edge.second));
                recursionStack.push(std::make_pair(edge.first, middle));
            } else {
                BOOST_ASSERT_MSG(!ed.shortcut, "edge must be a shortcut");
                unpackedPath.push_back(
                    _PathData(
                        ed.id,
                        facade->GetNameIndexFromEdgeID(ed.id),
                        facade->GetTurnInstructionForEdgeID(ed.id),
                        ed.distance
                    )
                );
            }
        }
    }

    inline void UnpackEdge(const NodeID s, const NodeID t, std::vector<NodeID> & unpackedPath) const {
        std::stack<std::pair<NodeID, NodeID> > recursionStack;
        recursionStack.push(std::make_pair(s,t));

        std::pair<NodeID, NodeID> edge;
        while(!recursionStack.empty()) {
            edge = recursionStack.top();
            recursionStack.pop();

            EdgeID smallestEdge = SPECIAL_EDGEID;
            int smallestWeight = INT_MAX;
            for(EdgeID eit = facade->BeginEdges(edge.first);eit < facade->EndEdges(edge.first);++eit){
                const int weight = facade->GetEdgeData(eit).distance;
                if(facade->GetTarget(eit) == edge.second && weight < smallestWeight && facade->GetEdgeData(eit).forward){
                    smallestEdge = eit;
                    smallestWeight = weight;
                }
            }

            if(smallestEdge == SPECIAL_EDGEID){
                for(EdgeID eit = facade->BeginEdges(edge.second);eit < facade->EndEdges(edge.second);++eit){
                    const int weight = facade->GetEdgeData(eit).distance;
                    if(facade->GetTarget(eit) == edge.first && weight < smallestWeight && facade->GetEdgeData(eit).backward){
                        smallestEdge = eit;
                        smallestWeight = weight;
                    }
                }
            }
            BOOST_ASSERT_MSG(smallestWeight != INT_MAX, "edge weight invalid");

            const typename DataFacadeT::EdgeData& ed = facade->GetEdgeData(smallestEdge);
            if(ed.shortcut) {//unpack
                const NodeID middle = ed.id;
                //again, we need to this in reversed order
                recursionStack.push(std::make_pair(middle, edge.second));
                recursionStack.push(std::make_pair(edge.first, middle));
            } else {
                BOOST_ASSERT_MSG(!ed.shortcut, "edge must be shortcut");
                unpackedPath.push_back(edge.first );
            }
        }
        unpackedPath.push_back(t);
    }

    inline void RetrievePackedPathFromHeap(
        SearchEngineData::QueryHeap & _fHeap,
        SearchEngineData::QueryHeap & _bHeap,
        const NodeID middle,
        std::vector<NodeID> & packedPath
    ) const {
        NodeID pathNode = middle;
        while(pathNode != _fHeap.GetData(pathNode).parent) {
            pathNode = _fHeap.GetData(pathNode).parent;
            packedPath.push_back(pathNode);
        }

        std::reverse(packedPath.begin(), packedPath.end());
        packedPath.push_back(middle);
        pathNode = middle;
        while (pathNode != _bHeap.GetData(pathNode).parent){
            pathNode = _bHeap.GetData(pathNode).parent;
            packedPath.push_back(pathNode);
    	}
    }

    inline void RetrievePackedPathFromSingleHeap(SearchEngineData::QueryHeap & search_heap, const NodeID middle, std::vector<NodeID>& packed_path) const {
        NodeID pathNode = middle;
        while(pathNode != search_heap.GetData(pathNode).parent) {
            pathNode = search_heap.GetData(pathNode).parent;
            packed_path.push_back(pathNode);
        }
    }
};


#endif /* BASICROUTINGINTERFACE_H_ */

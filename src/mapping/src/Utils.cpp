#include "mapping/Utils.h"

using namespace std;

vector<pair<int, int> > bresenhamLine( int x0, int y0, int x1, int y1 ) {
	vector<pair<int, int> > result;

	bool steep = abs(y1 - y0) > abs(x1 - x0);

	if( steep ) {
		SWAP( x0, y0 );
		SWAP( x1, y1 );
	}
	if( x0 > x1 ) {
		SWAP( x0, x1 );
		SWAP( y0, y1 );
	}

	int deltax = x1 - x0;
	int deltay = abs(y1 - y0);
	int error = deltax / 2;
	int y = y0;
	int ystep = ( y0 < y1 ) ? 1 : -1;

	for( int x = x0; x <= x1; x++ ) {
		if ( steep ) {
			// cell (y,x) is crossed
			result.push_back(pair<int, int>(y, x));
		} else {
			// cell (x,y) is crossed
			result.push_back(pair<int, int>(x, y));
		}

		error = error - deltay;
		if( error < 0 ) {
			y += ystep;
			error += deltax;
		}
	}
	
	return result;
}

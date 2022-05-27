#include "test_bench.h"
//#include "tsf/include/tsf.h"

int main(int argc, char* argv[])
{
	/*SetGPSBase(12.3, 45.6, 78.9);
	Point p;
	p.x = 1.0;
	p.y = 2.0;
	p.z = 3.0;

	Point result = GetWGS(p);
*/
	//cout << " p.x:" << p.x << " p.y:" << p.y << " p.z:" << p.z << endl;
	cout << "Hello World" << endl;
	const char* mv_id = "0";
	tester t(mv_id);

	bool isJoinTimeLoop = false;
	const char* serverIP = "127.0.0.1";
	t.Test_InitSimOneAPI(isJoinTimeLoop, serverIP);	
	t.Test_GPS(false);
	
	return 0;
}

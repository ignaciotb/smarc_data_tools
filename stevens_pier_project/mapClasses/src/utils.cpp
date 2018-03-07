#include "mapClasses/utils/utils.h"

namespace utils{

	float dotVec(std::vector<float> a, std::vector<float> b){
		return a[0]*b[0] + a[1]*b[1];
	}

	float lenVec(std::vector<float> a){
		return std::sqrt(a[0]*a[0] + a[1]*a[1]);
	}

	// void calcMeanVar(const std::vector<float> vec, float &ave, float &var){

	// 	float cnt = 0.0f;
	// 	float sum = 0.0f;
	// 	float cnt2= 0.0f;

	// 	for(size_t i=0; i < vec.size(); ++i){
	// 		cnt  += vec.at(i);
	// 		sum  += i*vec.at(i);
	// 		cnt2 += i*i*vec.at(i);
	// 	}
	// 	ave = sum/cnt;
	// 	var = cnt2/cnt - ave*ave;
	// }

	void calcMeanVar(const std::vector<float> vec, float &ave, float &var){

		float cnt = 0.0f;
		float sum = 0.0f;
		float cnt2= 0.0f;

		for(size_t i=0; i < vec.size(); ++i){
			cnt  += vec.at(i);
			sum  += i*vec.at(i);
			cnt2 += i*i*vec.at(i);
		}
		ave = sum/cnt;
		var = cnt2/cnt - ave*ave;
	}

}
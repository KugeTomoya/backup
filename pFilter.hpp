#ifndef PFILTER_HPP
#define PFILTER_HPP
#include <stdio.h>
#include <math.h>

using namespace cv;
using namespace std;

typedef struct {
	int x, y;
} NOISE;

class Pfilter{
private:
	bool risetFlag;
	int askB;
	int askG;
	int askR;
	vector<Particle*> particle;
	Mat *image;//image(画像データ)のポインタを持っていることがポイント
	Mat particle_img;
	Mat campus;
	NOISE noise;

public:
	Pfilter(Mat &image, NOISE noise, int b, int g, int r);
	void setParticle();
	void setWeight();
	void resample();
	void measure(Particle* result);


	void getParticle();
	bool getFlag();
};

#endif // PFILTER_HPP_INCLUDED

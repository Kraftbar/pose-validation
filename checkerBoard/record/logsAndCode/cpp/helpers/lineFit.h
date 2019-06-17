#pragma once
	
fitLine(){
    double sx = 0.0, sy = 0.0, stt = 0.0, sts = 0.0;
	int n = x.size();
	for (int i = 0; i < n; ++i)
	{
		sx += x[i];
		sy += y[i];
	}
	for (int i = 0; i < n; ++i)
	{
		double t = x[i] - sx/n;
		stt += t*t;
		sts += t*y[i];
	}

	double slope = sts/stt;
	double intercept = (sy - sx*slope)/n;
}

#include "main.h"
double boundRad(double rad){
  double res = fmod(rad, twoPI);
  if(res < 0) res += twoPI;
  return res;
}
double abscap(double x, double abscap){
  if(x > abscap) return abscap;
  else if(x < -abscap) return -abscap;
  else return x;
}
double distance(Node n1, Node n2){
  Node diff = n2 - n1;
  return diff.mag();
}
double circumRad(Node n1, Node n2, Node n3){
  double a = distance(n1, n2);
  double b = distance(n2, n3);
  double c = distance(n3, n1);
  double denom = sqrt((a+b+c)*(b+c-a)*(c+a-b)*(a+b-c));
  printf("Square: %.10f\n", ((a+b+c)*(b+c-a)*(c+a-b)*(a+b-c)));
  if(denom <= INFsmall) return INF;
  else return a*b*c/denom;
}
double isEqualDoubles(double a, double b, double epsilon = INFsmall){
  return fabs(a-b) <= epsilon;
}
double bearingToMathAngle(double angle){
  double res = halfPI - angle;
  return boundRad(res);
}

void waitUntil(bool condition) {while(!condition) delay(5);}
void waitUntil(bool condition, double cutoff) {
  double start = millis();
  while(!condition && (millis() - start) < cutoff) delay(5);
}

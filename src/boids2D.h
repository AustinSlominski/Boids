#pragma once
#include "ofMain.h"

const int ARR_LEN = 10;

class Boid : ofNode {
    
public:
    void run(vector<Boid> & boids);
    void update();

    //------ DRAW METHODS -------
    void drawTriangles();
    void drawEllipse();
    void drawTracer();
    void drawDebug();
    //---------------------------
    
    void applyForce(ofVec2f force);
    void flock(vector<Boid> & boids);
    bool checkDead(int offset);
    void pull(ofVec2f p, float strength, float limit);
    
    
    ofVec2f seek(ofVec2f target);
    ofVec2f align(vector<Boid> & boids);
    ofVec2f separate(vector<Boid> & boids);
    ofVec2f cohesion(vector<Boid> & boids);
    
    ofVec3f accumulator[ARR_LEN];
    ofVec3f running_avg, previous_avg;
    int curr_ind, prev_ind;

    float theta;
    float maxspeed, maxforce;
    float color_scl;
    
    bool b_color;
    
    int draw_size;
    int tracer_length;

    ofFloatColor color;
    ofPoint position;
    ofVec2f velocity, acceleration;
    ofVec2f coh, ali, sep;
    
    string error;
    ofMesh tracer;
    
    Boid(int x, int y);
};

#include "boids2D.h"

//--------------------------------------------------------------
Boid::Boid(int x, int y)
{
    position     = ofVec2f(x,y);
    velocity     = ofVec2f(ofRandomf(),ofRandomf());
    acceleration = ofVec2f(0,0);
    color        = ofFloatColor(1,1,1);
    
    maxspeed = 2.00;
    maxforce = 0.03;
    
    color_scl = 400;
    
    memset(accumulator, 0.0, sizeof(accumulator));

    p_ind = 0;
    c_ind = 0;
    
    b_color = true;
    draw_size = 4;
    
    tracer_length = 300;
}

//--------------------------------------------------------------
void Boid::run(vector<Boid> & boids)
{
    flock(boids);
    update();

    drawTracer();
    
    ofPushMatrix();
        ofTranslate(position.x,position.y);
        ofRotate(theta);
        drawTriangles();
    ofPopMatrix();
}

//--------------------------------------------------------------
void Boid::update()
{
    if(b_color){
        //-------------------------------------
        //---------CIRCULAR BUFFER-------------
        //-------------------------------------
        accumulator[c_ind] = ofVec3f(ali.lengthSquared(),sep.lengthSquared(),coh.lengthSquared()) * velocity.lengthSquared();
        if(c_ind % ARR_LEN){
            ofVec3f total = accumulate(begin(accumulator), end(accumulator), ofVec3f(0.0,0.0,0.0));
            previous_avg = running_avg;
            running_avg = total/(float)ARR_LEN;
        }else{
            running_avg = previous_avg.interpolate(running_avg,((float)c_ind/(float)ARR_LEN));
        }
        c_ind = (c_ind + 1) % ARR_LEN;
        p_ind = (c_ind - 1) % ARR_LEN;
        //-------------------------------------
        
        
        color = ofFloatColor(1-(running_avg.y*color_scl),1-(running_avg.x*color_scl),1-(running_avg.z*color_scl));
    }else{
        color = ofFloatColor(255);
    }
    
    //update velocity
    velocity = acceleration + velocity;
    //range speed
    velocity.limit(maxspeed);
    position = position     + velocity;
    acceleration = ofVec2f(0,0);
    
    ofVec2f direction = velocity;
    theta = ofRadToDeg(atan2(direction.y,direction.x))-90.0;
}

//--------------------------------------------------------------
void Boid::drawTriangles(){

    ofSetColor(color);
        ofDrawTriangle(0,draw_size*2,-draw_size,-draw_size,draw_size,-draw_size);
    ofSetColor(255);

}

//--------------------------------------------------------------
void Boid::drawEllipse(){
    ofSetColor(color);
    ofNoFill();
    ofDrawEllipse(0,0,draw_size,draw_size*2);
    ofFill();
    ofSetColor(255);
}

//--------------------------------------------------------------
void Boid::drawDebug(){
    ofSetColor(ofColor(255,0,0));
    ofDrawBitmapString(error,0,-20);
}

//--------------------------------------------------------------
void Boid::drawTracer(){
    tracer.setMode(OF_PRIMITIVE_LINE_STRIP);
    glLineWidth(2);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);
    
    if(tracer.getNumVertices() > tracer_length){
        tracer.getVertices().erase(tracer.getVertices().begin());
    }
    
    tracer.addVertex(ofVec3f(position.x,position.y,0));
    tracer.addColor(ofFloatColor(color.r,color.g,color.b,1));
    
    if(tracer.getNumVertices() > 0){
        for(int i = 0; i < tracer.getNumVertices(); i++){
            float t_col = float(i)/float(tracer.getNumVertices());
            tracer.setColor(i,ofFloatColor(tracer.getColor(i),t_col));
        }
    }
    
    ofSetColor(255,255,255);
    tracer.draw();
}

//--------------------------------------------------------------
void Boid::applyForce(ofVec2f force){
    // We could add mass here if we want A = F / M
    acceleration = acceleration + force;
}

//--------------------------------------------------------------
void Boid::flock(vector<Boid> & boids) {
    sep = separate(boids);   // Separation
    ali = align(boids);      // Alignment
    coh = cohesion(boids);   // Cohesion
    // Arbitrarily weight these forces
    sep = sep * 1.5;
    ali = ali * 1.0;
    coh = coh * 1.0;
    // Add the force vectors to acceleration
    applyForce(sep);
    applyForce(ali);
    applyForce(coh);
}

//--------------------------------------------------------------
ofVec2f Boid::seek(ofVec2f target){
    ofVec2f desired = target - position;
    // Scale to maximum speed
    desired.normalize();
    desired = desired * maxspeed;

    // Steering = Desired minus Velocity
    ofVec2f steer = desired - velocity;
    steer.limit(maxforce);  // Limit to maximum steering force
    return steer;
}

//--------------------------------------------------------------
ofVec2f Boid::separate(vector<Boid> & boids){
    //distance of desired separation
    float desiredseparation = 25.0f;
    
    //steering vector
    ofVec2f steer = ofVec2f(0, 0);
    
    for (int i=0; i < boids.size(); i++) {
        float d = position.distance(boids[i].position);
        
        if ((d > 0) && (d < desiredseparation)) {
            ofVec2f diff = position - boids[i].position;
            diff.normalize();
            diff = diff/d;
            steer = steer+diff;
        }
    }
    
    if (boids.size() > 0) {
        steer = steer/boids.size();
    }
    
    if (sqrt(steer.x*steer.x + steer.y*steer.y) > 0) {
        steer.normalize();
        steer = steer * maxspeed;
        steer = steer - velocity;
        steer.limit(maxforce);
    }
    return steer;
}

//--------------------------------------------------------------
ofVec2f Boid::align(vector<Boid> & boids){
    float neighbordist = 50;
    ofVec2f sum = ofVec2f(0, 0);
    int count = 0;
    for (int i=0; i < boids.size(); i++) {
        float d = position.distance(boids[i].position);
        if ((d > 0) && (d < neighbordist)) {
            sum = sum + boids[i].velocity;
            count++;
        }
    }
    if (count > 0) {
        sum = sum/(float)count;
        // Implement Reynolds: Steering = Desired - Velocity
        sum.normalize();
        sum = sum * maxspeed;
        ofVec2f steer = sum - velocity;
        steer.limit(maxforce);
        return steer;
    }
    else {
        return ofVec2f(0, 0);
    }
}

//--------------------------------------------------------------
ofVec2f Boid::cohesion(vector<Boid> & boids){
    float neighbordist = 50;
    ofVec2f sum = ofVec2f(0, 0);
    int count = 0;
    for (int i=0; i < boids.size(); i++) {
        float d = position.distance(boids[i].position);
        if ((d > 0) && (d < neighbordist)) {
            sum = sum + boids[i].position; // Add position
            count++;
        }
    }
    if (count > 0) {
        sum = sum/count;
        return seek(sum);  // Steer towards the position
    }
    else {
        return ofVec2f(0,0);
    }
}

//--------------------------------------------------------------
bool Boid::checkDead(){
    if(position.x > ofGetWidth()+700 || position.x < 0-700 || position.y > ofGetHeight()+700 || position.y < 0-700){
        return true;
    }else{
        return false;
    }
}

//--------------------------------------------------------------
void Boid::pull(ofVec2f p, float strength, float range){

    ofVec2f delta = p-position;
    float theta = atan2(delta.y,delta.x);
    ofVec2f tmp_point = ofVec2f(1,0);
    ofVec2f pull_vec = tmp_point.rotateRad(theta);
    
    ofVec2f diff = p-position;
    float distance = diff.length();
    
    if(distance <= range){
        float n_distance = distance/range;

        //sin((freq*PI)*(distance-(offset)))/2
        float influence = 0.5 + sin((2*PI)*(n_distance-(0.25)))/2;
        
        ofVec2f t_pull = pull_vec*influence*strength;
        t_pull.limit(strength);
        applyForce(t_pull);
    }
}

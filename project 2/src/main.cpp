#include <iostream>
#include <chrono>

#include <GL/glut.h>

#include "I2CBus.h"
#include "MinIMU9.h"
#include "MadgwickFilter.h"

// ==============================================================================
// Globals (kept here for GLUT callbacks)
// ==============================================================================
static I2CBus* i2cBus = nullptr;
static MinIMU9* imu = nullptr;
static MadgwickFilter* filter = nullptr;

static auto lastTime = std::chrono::steady_clock::now();

static void buildRotationMatrix(float *matrix, float w, float x, float y, float z) {
    matrix[0]  = 1.0f - 2.0f * (y*y + z*z); matrix[1]  = 2.0f * (x*y + w*z);       matrix[2]  = 2.0f * (x*z - w*y);       matrix[3]  = 0.0f;
    matrix[4]  = 2.0f * (x*y - w*z);       matrix[5]  = 1.0f - 2.0f * (x*x + z*z); matrix[6]  = 2.0f * (y*z + w*x);       matrix[7]  = 0.0f;
    matrix[8]  = 2.0f * (x*z + w*y);       matrix[9]  = 2.0f * (y*z - w*x);       matrix[10] = 1.0f - 2.0f * (x*x + y*y); matrix[11] = 0.0f;
    matrix[12] = 0.0f;                     matrix[13] = 0.0f;                     matrix[14] = 0.0f;                     matrix[15] = 1.0f;
}

static void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glTranslatef(0.0f, 0.0f, -5.0f);

    float rotMatrix[16];
    buildRotationMatrix(rotMatrix, filter->q0, filter->q1, filter->q2, filter->q3);
    glMultMatrixf(rotMatrix);

    glBegin(GL_LINES);
        glColor3f(1.0f, 0.0f, 0.0f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(2.0f, 0.0f, 0.0f);
        glColor3f(0.0f, 1.0f, 0.0f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.0f, 2.0f, 0.0f);
        glColor3f(0.0f, 0.0f, 1.0f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.0f, 0.0f, 2.0f);
    glEnd();

    glColor3f(0.8f, 0.8f, 0.8f);
    glutWireCube(1.5);

    glutSwapBuffers();
}

static void idle() {
    float ax, ay, az, gx, gy, gz, mx, my, mz;

    auto currentTime = std::chrono::steady_clock::now();
    std::chrono::duration<float> elapsed = currentTime - lastTime;
    float dt = elapsed.count();
    lastTime = currentTime;

    imu->readData(ax, ay, az, gx, gy, gz, mx, my, mz);
    filter->update(gx, gy, gz, ax, ay, az, mx, my, mz, dt);

    glutPostRedisplay();
}

static void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (float)w / (float)h, 0.1f, 100.0f);
    glMatrixMode(GL_MODELVIEW);
}

static void cleanup() {
    delete imu; imu = nullptr;
    delete i2cBus; i2cBus = nullptr;
    delete filter; filter = nullptr;
}

int main(int argc, char** argv) {
    i2cBus = new I2CBus("/dev/i2c-1");
    imu = new MinIMU9(i2cBus);
    filter = new MadgwickFilter();

    imu->init();
    lastTime = std::chrono::steady_clock::now();

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Real-Time Hardware IMU 3D Visualization");

    glEnable(GL_DEPTH_TEST);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutIdleFunc(idle);

    atexit(cleanup);

    std::cout << "Starting hardware loop... Rotate the sensor to move the cube.\n";
    glutMainLoop();
    return 0;
}
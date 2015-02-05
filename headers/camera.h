#ifndef CAMERA_H
#define CAMERA_H

#include "matrix.h"


class Camera{
private:
    bool m_moved;
    float m_speed;
    float m_yaw_speed;
    float m_pos[3];
    float m_yaw;
    float m_near;
    float m_far;
    float m_fov;
    float m_width;
    float m_height;
    float m_aspect;
    float m_range;
    float m_Sx;
    float m_Sy;
    float m_Sz;
    float m_Pz;


public:
    Camera(float _width, float _height)
    {
     m_speed = 1.0f;
     m_yaw_speed = 10.0f;
     m_pos[0] = 0.0f;
     m_pos[1] = 0.0f;
     m_pos[2] = 2.0f;
     m_yaw = 0.0f;
     m_near = 0.1f;
     m_far = 100.0f;
     m_fov = 67.0f * ONE_DEG_IN_RAD;
     m_width = _width;
     m_height = _height;
     calculateConfiguration();
    }

    ~Camera() {}


    void calculateConfiguration()
    {
        m_aspect = (float)m_width/(float)m_height;
        m_range = tan(m_fov*0.5f)*m_near;
        m_Sx = (2.0f * m_near) / (m_range * m_aspect + m_range * m_aspect);
        m_Sy = m_near/m_range;
        m_Sz = -(m_far + m_near)/(m_far - m_near);
        m_Pz = -(2.0f * m_far * m_near)/(m_far - m_near);
    }

    void translate(int i, float _value)
    {
        m_pos[i] += _value;
    }

    void changeSpeed(float _value)
    {
        m_speed += _value;
    }

    void changeYawSpeed(float _value)
    {
        m_yaw_speed += _value;
    }

    void changeYaw(float _value)
    {
      m_yaw += _value;
    }

    //Setters and getters
    void setMoved(bool _moved) {m_moved = _moved;}
    bool getMoved() { return m_moved;}
    void setSpeed(float _speed) {m_speed = _speed;}
    float getSpeed(){ return m_speed;}
    void setYawSpeed(float _yaw_speed) {m_yaw_speed = _yaw_speed;}
    float getYawSpeed(){ return m_yaw_speed;}
    void setPosition(float _x, float _y, float _z)
    {
        m_pos[0] = _x;
        m_pos[1] = _y;
        m_pos[2] = _z;
    }

    float* getPosition() {return m_pos;}
    float x() {return m_pos[0];}
    float y() {return m_pos[1];}
    float z() {return m_pos[2];}
    void setYaw(float _yaw) { m_yaw = _yaw;}
    float getYaw() { return m_yaw; }
    void setNear(float _near) { m_near = _near; }
    bool getNear() { return m_near; }
    void setFar(float _far) { m_far = _far;}
    bool getFar() { return m_far; }
    void setSx(float _Sx) { m_Sx = _Sx; }
    float getSx() { return m_Sx; }
    void setSy(float _Sy) { m_Sy = _Sy; }
    float getSy() { return m_Sy; }
    void setSz(float _Sz) { m_Sz = _Sz; }
    float getSz() { return m_Sz; }
    void setPz(float _Pz) { m_Pz = _Pz; }
    float getPz() { return m_Pz; }
};
#endif // CAMERA_H

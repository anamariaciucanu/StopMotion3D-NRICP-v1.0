#ifndef CAMERA_H
#define CAMERA_H

///@file Camera.h
///@brief Defines the camera class with all the configuration details
///@author Anamaria Ciucanu


#include "matrix.h"
#include "math.h"


class Camera
{
 private:
    //fdsffsdfsdfd
    ///@brief Boolean variable -> checks whether the camera has moved using the keyboard buttons
    bool m_moved;
    ///@brief Boolean variable -> checks whether the camera has rotated using the keyboard buttons
    bool m_rotated;
    ///@brief Float variable -> speed of the camera when moving
    float m_speed;
    ///@brief Float variable -> yaw speed of the camera (rotation around Y axis)
    float m_yaw_speed;
    ///@brief Array of 3 floats -> position of the camera in world space
    float m_pos[3];
    ///@brief Float variable -> yaw angle of the camera
    float m_yaw;
    ///@brief Float variable -> near plane of the camera
    float m_near;
    ///@brief Float variable -> far plane of the camera
    float m_far;
    ///@brief Float variable -> field of view angle in degrees
    float m_fov;
    ///@brief Float variable -> width of the image plane
    float m_width;
    ///@brief Float variable -> height of the image plane
    float m_height;
    ///@brief Float variable -> aspect ratio
    float m_aspect;
    ///@brief Float variable -> distance between near and far planes
    float m_range;
    ///@brief Float variables -> parameters for the projection matrix
    float m_Sx;
    float m_Sy;
    float m_Sz;
    float m_Pz;


public:

///@brief ctor for Camera class
///@param [in] _width -> width of the image plane
///@param [in] _height -> height of the image plane
    Camera(float _width, float _height)
    {
     m_moved = false;
     m_rotated = false;
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

///@brief dtor for Camera class
    ~Camera() {}

///@brief calculates the parameters to be included in the projection matrix
    void calculateConfiguration()
    {
        m_aspect = (float)m_width/(float)m_height;
        m_range = tan(m_fov*0.5f)*m_near;
        m_Sx = (2.0f * m_near) / (m_range * m_aspect + m_range * m_aspect);
        m_Sy = m_near/m_range;
        m_Sz = -(m_far + m_near)/(m_far - m_near);
        m_Pz = -(2.0f * m_far * m_near)/(m_far - m_near);
    }

///@brief translates the camera per axis
///@param [in] _i -> x, y, z axis index
///@param [in] _value -> translation amount
    void translate(int _i, float _value)
    {
        if(_i<3)
        {
         m_pos[_i] += _value;
        }
    }

///@brief changes the speed of the camera when moving through the scene
///@param [in] _value -> amount of speed to be added/decreased from the current speed
    void changeSpeed(float _value)
    {
        m_speed += _value;
    }

 ///@brief changes the yaw speed of the camera when rotating around Y axis
 ///@param [in] _value -> amount of yaw speed to be added/decreased from the current yaw speed
    void changeYawSpeed(float _value)
    {
        m_yaw_speed += _value;
    }

 ///@brief changes the yaw angle
 ///@param [in] _value -> the angle to be added/decreased from the current yaw angle
    void changeYaw(float _value)
    {
      m_yaw += _value;
    }

 ///@brief Setters and Getters for the private variables
    void setMoved(bool _moved) { m_moved = _moved; }
    bool isMoved() { return m_moved; }
    void setRotated(bool _rotated) { m_rotated = _rotated; }
    bool isRotated() { return m_rotated; }
    void setSpeed(float _speed) { m_speed = _speed; }
    float getSpeed(){ return m_speed; }
    void setYawSpeed(float _yaw_speed) { m_yaw_speed = _yaw_speed; }
    float getYawSpeed(){ return m_yaw_speed; }
    void setPosition(float _x, float _y, float _z)
    {
        m_pos[0] = _x;
        m_pos[1] = _y;
        m_pos[2] = _z;
    }
    float* getPosition() { return m_pos;}
    float x() { return m_pos[0]; }
    float y() { return m_pos[1]; }
    float z() { return m_pos[2]; }
    void setYaw(float _yaw) { m_yaw = _yaw; }
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

//=============================================================================
//
//   Exercise code for the lecture
//   "Advanced Computer Graphics"
//
//   Adapted from Prof. Dr. Mario Botsch, Bielefeld University
//
//   Copyright (C) 2013 LGG, epfl
//
//   DO NOT REDISTRIBUTE
//=============================================================================


//== INCLUDES =================================================================

#include "Rigid_body_viewer.h"
#include <utils/gl.h>


//== IMPLEMENTATION ========================================================== 


Rigid_body_viewer::Rigid_body_viewer(const char* _title, int _width, int _height)
    : Viewer_2D(_title, _width, _height)
{
    animate_             = false;
    time_step_           = 0.001;
    mass_                = 0.5;
    damping_linear_      = 0.1;
    damping_angular_     = 0.0001;
    spring_stiffness_    = 100.0;
    spring_damping_      = 5.0;

    mouse_spring_.active = false;
}


//-----------------------------------------------------------------------------


void Rigid_body_viewer::keyboard(int key, int x, int y)
{
    switch (key)
    {
    case '1':
    {
        std::vector<vec2> p;
        p.push_back( vec2(-0.6, -0.6) );
        p.push_back( vec2(-0.4, -0.6) );
        p.push_back( vec2(-0.4, -0.4) );
        p.push_back( vec2(-0.6, -0.4) );

        body_ = Rigid_body(p, mass_);
        body_.linear_velocity = vec2(5.0, 5.0);
        glutPostRedisplay();
        break;
    }
    case '2':
    {
        std::vector<vec2> p;
        p.push_back( vec2(-0.3, -0.1) );
        p.push_back( vec2(-0.1, -0.1) );
        p.push_back( vec2( 0.1, -0.1) );
        p.push_back( vec2( 0.3, -0.1) );
        p.push_back( vec2( 0.3,  0.1) );
        p.push_back( vec2( 0.1,  0.1) );
        p.push_back( vec2(-0.1,  0.1) );
        p.push_back( vec2(-0.3,  0.1) );

        body_ = Rigid_body(p, mass_);

        glutPostRedisplay();
        break;
    }
    case '3':
    {
        std::vector<vec2> p;
        p.push_back( vec2(-0.5,  0.1) );
        p.push_back( vec2(-0.5,  0.0) );
        p.push_back( vec2( 0.0,  0.0) );
        p.push_back( vec2( 0.0, -0.3) );
        p.push_back( vec2( 0.1, -0.3) );
        p.push_back( vec2( 0.1,  0.0) );
        p.push_back( vec2( 0.3,  0.0) );
        p.push_back( vec2( 0.3,  0.1) );

        body_ = Rigid_body(p, mass_);

        glutPostRedisplay();
        break;
    }
        // let parent class do the work
    default:
    {
        Viewer_2D::keyboard(key, x, y);
        break;
    }
    }
}


//-----------------------------------------------------------------------------


void Rigid_body_viewer:: mouse(int _button, int _state, int _x, int _y)
{
    // need points
    if (body_.points.empty())
        return;

    // mouse button release destroys current mouse spring
    if (_state == GLUT_UP)
    {
        mouse_spring_.active = false;
        return;
    }

    // mouse button press generates new mouse spring
    else if (_state == GLUT_DOWN)
    {
        // get point under mouse cursor
        vec2 p = pick(_x, _y);

        // find closest body point
        unsigned int i, imin;
        float dmin = FLT_MAX;
        for (i=0; i<body_.points.size(); ++i)
        {
            float d = distance(p, body_.points[i]);
            if (d < dmin)
            {
                dmin = d;
                imin = i;
            }
        }

        // setup the mouse spring
        mouse_spring_.active = true;
        mouse_spring_.particle_index = imin;
        mouse_spring_.mouse_position = p;
    }

    glutPostRedisplay();
}


//-----------------------------------------------------------------------------


void Rigid_body_viewer:: motion(int _x, int _y)
{
    if (mouse_spring_.active)
    {
        // update mouse position
        mouse_spring_.mouse_position = pick(_x, _y);
        glutPostRedisplay();
    }
}


//-----------------------------------------------------------------------------


void Rigid_body_viewer:: draw()
{
    // parent's status text
    Viewer_2D::draw();

    // draw walls
    glDisable(GL_LIGHTING);
    glLineWidth(1.0);
    glColor3f(0.5,0.5,0.5);
    glBegin(GL_LINE_STRIP);
    glVertex2f( -1.0,  1.0 );
    glVertex2f( -1.0, -1.0 );
    glVertex2f(  1.0, -1.0 );
    glVertex2f(  1.0,  1.0 );
    glVertex2f( -1.0,  1.0 );
    glEnd();

    // draw rigid body
    body_.draw();

    // draw mouse spring
    if (mouse_spring_.active)
    {
        glLineWidth(5.0);
        glColor3f(1,0,0);
        glBegin(GL_LINES);
        glVertex2fv( body_.points[ mouse_spring_.particle_index ].data() );
        glVertex2fv( mouse_spring_.mouse_position.data() );
        glEnd();
    }
}


//-----------------------------------------------------------------------------


void Rigid_body_viewer::compute_forces()
{ 
    /** \todo Compute all forces acting on the rigid body
     \li clear all forces
     \li add gravity
     \li add damping to linear and angular movement
     \li add the mouse spring force
     */


    const float gravity = 9.81;
    vec2 gravityForce = -body_.mass * vec2(0, gravity);
    vec2 linearDamp = -body_.linear_velocity * damping_linear_;
    float angularDamp = -body_.angular_velocity * damping_angular_;

    body_.force = gravityForce + linearDamp;
    body_.torque = angularDamp;

    if (mouse_spring_.active) {
        vec2 bodyPos = body_.points[mouse_spring_.particle_index];
        vec2 mousePos = mouse_spring_.mouse_position;
        float springL = norm(bodyPos - mousePos);
        vec2 spring = bodyPos - mousePos;
        vec2 springForce = -(spring_stiffness_*springL + spring_damping_*dot(body_.linear_velocity, spring)/springL)*spring/springL;

        vec2 rLocal= body_.r[mouse_spring_.particle_index];
        float c = cos(body_.orientation);
        float s = sin(body_.orientation);
        vec2 rGlobal = vec2(rLocal[0]*c + rLocal[1]*s, - rLocal[0]*s + rLocal[1]*c);
        float springTorque = dot(perp(rGlobal), springForce);

        body_.force += springForce;
        body_.torque += springTorque;
    }

}


//-----------------------------------------------------------------------------

inline vec2 cross(float w, vec2 r) {
    // w x r = (0,0,w) x (r_x, r_y, 0) = (-w r_y, w r_x, 0)
    return vec2(-w*r[1], w*r[0]);
}

inline float cross(vec2 a, vec2 b) {
    // a x b = a_x b_y - a_y b_x
    return a[0]*b[1] - a[1]*b[0];
}

void Rigid_body_viewer::impulse_based_collisions()
{
    /** \todo Handle collisions based on impulses
     */
    float planes[4][3] = {
            {  0.0,  1.0, 1.0 },
            {  0.0, -1.0, 1.0 },
            {  1.0,  0.0, 1.0 },
            { -1.0,  0.0, 1.0 }
    };

    for (size_t plane_i = 0; plane_i < 4; ++plane_i)
    {
        for (unsigned int i = 0; i < body_.points.size(); i++ )
        {
            float d = planes[plane_i][0] * body_.points[i][0] +
                  planes[plane_i][1] * body_.points[i][1] +
                  planes[plane_i][2];

            if (d < 0.f) {
                float e = 0.5f; // Between 0 and 1

                vec2 n = vec2 (planes[plane_i][0], planes[plane_i][1]);
                float c = cos(body_.orientation);
                float s = sin(body_.orientation);
                vec2 r = body_.r[i];
                r = - vec2( c*r[0] + s*r[1],
                            -s*r[0] + c*r[1]);
                float m = body_.mass;
                float I = body_.inertia;


                vec2 v1 = body_.linear_velocity;
                float w1 = body_.angular_velocity;

                vec2 v1_p = v1 + cross(w1, r);

                float tmp = cross(r, n);
                float j = -(1 + e)*dot(v1_p, n) / (1/m + tmp*tmp/I);

                vec2 v2 = v1 + j*n/m;
                float w2 = w1 + cross(r, j*n) / I;

                body_.linear_velocity = v2;
                body_.angular_velocity = w2;

                body_.position -= d*n;
            }
        }
  }
}



//-----------------------------------------------------------------------------


void Rigid_body_viewer::time_integration(float dt)
{
    // compute all forces
    compute_forces();

    /** \todo Implement explicit Euler time integration
     \li update position and orientation
     \li update linear and angular velocities
     \li call update_points() at the end to compute the new particle positions
     */
    // handle collisions
    impulse_based_collisions();

    vec2 linearAccel = body_.force/body_.mass;
    float angularAccel = body_.torque/body_.inertia;

    body_.position += body_.linear_velocity * dt;
    body_.linear_velocity += linearAccel * dt;

    body_.orientation += body_.angular_velocity * dt;
    body_.angular_velocity += angularAccel * dt;

    body_.update_points();
}


//=============================================================================

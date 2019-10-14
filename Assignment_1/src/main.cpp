#include <iostream>
#include <string>
#include <vector>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "utils.h"
#include <cmath>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;
using namespace Eigen;
struct Sphere{
    double s_r;
    Vector3d s_c;
};

Vector3d S_intersection(double s_r,Vector3d s_c,Vector3d e,Vector3d d)
{
    double A = (d.norm())*(d.norm());
    double B =  2*d.dot(e-s_c);
    double C =  (e-s_c).squaredNorm() - pow(s_r,2);
    double D = pow(B,2) - 4*A*C;
    if(D<0)
    {
        return s_c;
    }
    else{
        double t;
        double t1 = (-B + sqrt(D))/(2*A);
        double t2 = (-B - sqrt(D))/(2*A);
        if(t1>0 && t2>0){
             t = (t1>t2?t1:t2);
        }
        else if(t1>0){
            t = t1;
        }
        else if(t2>0){
            t = t2;
        }
        else{
            return s_c;
        }
        return (e+t*d);
    }
}

Vector3d T_intersection(Vector3d e,Vector3d d,Vector3d a,Vector3d b,Vector3d c)
{
    Vector3d A = b-a;
    Vector3d B = c-a;
    Vector3d C = e-a;
    Vector3d n = A.cross(B);
    Vector3d q = C.cross(d);
    double D = 1./(d.dot(n));
    double u = D*(-q.dot(B));
    double v = D*(q.dot(A));
    double t = D*(-n.dot(C));
    if(u<0 || v<0 || (u+v)>1)
    {
        t = -1;
    }
    return Vector3d(t,u,v);
}

void ray_tracing_spheres()
{
    cout<<"Simple ray tracing for multiple spheres"<<endl;
    const string filename("spheres.png");
    MatrixXd R = MatrixXd::Constant(800,800,0.0); // Store the color red
    MatrixXd G = MatrixXd::Constant(800,800,0.0); // Store the color green
    MatrixXd B = MatrixXd::Constant(800,800,0.0); // Store the color blue
    MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

    //Camera 
    Vector3d o(-1,1,1);
    Vector3d x_disp(2.0/R.cols(),0,0);
    Vector3d y_disp(0,-2.0/R.rows(),0);

    // Single light source
    const Vector3d light1(0.3,0.2,-2);

    Sphere s1,s2;
    s1.s_r = 0.4;
    s2.s_r = 0.3;
    s1.s_c = RowVector3d(-0.4,0.1,0);
    s2.s_c = RowVector3d(0.3,0.2,0);

    for(int i=0;i<R.rows();i++)
    {
        for(int j=0;j<R.cols();j++)
        {
            // viewing ray
            Vector3d ray_o = o + double(i)*x_disp + double(j)*y_disp;
            Vector3d ray_dir = RowVector3d(0,0,-1);
            //intersection
            Vector3d S_intersect1 = S_intersection(s1.s_r,s1.s_c,ray_o,ray_dir);
            Vector3d S_intersect2 = S_intersection(s2.s_r,s2.s_c,ray_o,ray_dir);
            //check intersection
            if(S_intersect1 != s1.s_c)
            {
                Vector3d ray_n = (s1.s_c - S_intersect1).normalized(); //ray normal
                //diffuse shading
                R(i,j) = (S_intersect1 - light1).normalized().dot(ray_n);
                R(i,j) = 0.5*max(R(i,j),0.0);
                G(i,j) = B(i,j) = R(i,j);
                //sphere color=red
                R(i,j) = max(min(R(i,j),1.),0.);
                G(i,j) = max(min(G(i,j),0.),0.);
                B(i,j) = max(min(B(i,j),0.),0.);
                A(i,j) = 1;
            }

            if(S_intersect2 != s2.s_c)
            {
                Vector3d ray_n = (s2.s_c - S_intersect2).normalized(); //ray normal
                //diffuse shading
                R(i,j) = 0.5*(S_intersect2 - light1).normalized().dot(ray_n);
                R(i,j) = max(R(i,j),0.0);
                G(i,j) = B(i,j) = R(i,j);
                //sphere color=blue
                R(i,j) = max(min(R(i,j),0.),0.);
                G(i,j) = max(min(G(i,j),0.),0.);
                B(i,j) = max(min(B(i,j),1.),0.);
                A(i,j) = 1;
            }
            
        }
    }
    // Save to png
    write_matrix_to_png(R,G,B,A,filename);
}

void shading()
{
    cout<<"Specular and Ambient shading for multiple spheres, color and addition of 2nd light source"<<endl;
    const string filename1("color.png");
    MatrixXd R = MatrixXd::Constant(800,800,0.0); // Store the color red
    MatrixXd G = MatrixXd::Constant(800,800,0.0); // Store the color green
    MatrixXd B = MatrixXd::Constant(800,800,0.0); // Store the color blue
    MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask
    //Camera 
    Vector3d o(-1,1,1);
    Vector3d x_disp(2.0/R.cols(),0,0);
    Vector3d y_disp(0,-2.0/R.rows(),0);

    // Single light source
    const Vector3d light1(0.3,-0.2,-2);
    const Vector3d light2(-1,-1,-2);

    Sphere s1,s2;
    s1.s_r = 0.4;
    s2.s_r = 0.3;
    s1.s_c = RowVector3d(-0.4,0.1,0);
    s2.s_c = RowVector3d(0.3,0.2,0);

    for(int i=0;i<R.rows();i++)
    {
        for(int j=0;j<R.cols();j++)
        {
            // viewing ray
            Vector3d ray_o = o + double(i)*x_disp + double(j)*y_disp;
            Vector3d ray_dir = RowVector3d(0,0,-1);
            //intersection
            Vector3d S_intersect1 = S_intersection(s1.s_r,s1.s_c,ray_o,ray_dir);
            Vector3d S_intersect2 = S_intersection(s2.s_r,s2.s_c,ray_o,ray_dir);
            const double p = 1000; //phong coeff
            //check intersection
            if(S_intersect1 != s1.s_c)
            {
                Vector3d ray_n = (s1.s_c - S_intersect1).normalized(); //ray normal
                //diffuse shading
                R(i,j) = (S_intersect1 - light1).normalized().dot(ray_n);
                R(i,j) = 0.5*max(R(i,j),0.0);
                R(i,j) += 0.5*max((S_intersect1 - light2).normalized().dot(ray_n),0.);
                //R(i,j) = max(R(i,j),0.0);
                //phong shading
                Vector3d v = S_intersect1 - ray_o;
                Vector3d l1 = S_intersect1 - light1;
                Vector3d l2 = S_intersect1 - light2;
                Vector3d h1 = (v + l1).normalized(); //half vector 1
                Vector3d h2 = (v + l2).normalized(); //half vector 2
                double spec1 = (ray_n).normalized().dot(h1);
                double spec2 = (ray_n).normalized().dot(h2);
                spec1 = max(spec1,0.0);
                spec1 = pow(spec1,p);
                spec2 = max(spec2,0.0);
                spec2 = pow(spec2,p);
                R(i,j) += spec1;
                R(i,j) += spec2;
                G(i,j) = B(i,j) = R(i,j);
                //sphere color=red
                R(i,j) = max(min(R(i,j),1.),0.);
                G(i,j) = max(min(G(i,j),0.),0.);
                B(i,j) = max(min(B(i,j),0.),0.);
                A(i,j) = 1;
            }

            if(S_intersect2 != s2.s_c)
            {
                Vector3d ray_n = (s2.s_c - S_intersect2).normalized(); //ray normal
                //diffuse shading
                R(i,j) = (S_intersect2 - light2).normalized().dot(ray_n); 
                R(i,j) = 0.5*max(R(i,j),0.0);
                R(i,j) += (S_intersect2 - light2).normalized().dot(ray_n);
                R(i,j) = 0.5*max(R(i,j),0.0);
                //phong shading
                Vector3d h1 = (light1 + s2.s_c - ray_o).normalized(); //half vector 1
                Vector3d h2 = (light2 + s2.s_c - ray_o).normalized(); //half vector 2
                double spec1 = (ray_n).normalized().dot(h1);
                double spec2 = (ray_n).normalized().dot(h2);
                spec1 = max(spec1,0.0);
                spec1 = pow(spec1,p);
                spec2 = max(spec2,0.0);
                spec2 = pow(spec2,p);
                R(i,j) += spec1;
                R(i,j) += spec2;
                R(i,j) += 0.3;
                G(i,j) = B(i,j) = R(i,j);
                //sphere color=blue
                R(i,j) = max(min(R(i,j),0.),0.);
                G(i,j) = max(min(G(i,j),0.),0.);
                B(i,j) = max(min(B(i,j),1.),0.);
                A(i,j) = 1;
            }
        }
    }
    // Save to png
    write_matrix_to_png(R,G,B,A,filename1);
}

void perspective_proj()
{
    cout<<"Perspective projection"<<endl;
    const string filename("perspective.png");
    MatrixXd R = MatrixXd::Constant(800,800,0.0); // Store the color red
    MatrixXd G = MatrixXd::Constant(800,800,0.0); // Store the color green
    MatrixXd B = MatrixXd::Constant(800,800,0.0); // Store the color blue
    MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask
    //Camera 
    Vector3d o(-1,1,1);
    Vector3d x_disp(2.0/R.cols(),0,0);
    Vector3d y_disp(0,-2.0/R.rows(),0);

    // Single light source
    const Vector3d light1(0.3,-0.2,-2);
    const Vector3d light2(-1,-1,-2);

    Sphere s1,s2;
    s1.s_r = 0.4;
    s2.s_r = 0.3;
    s1.s_c = RowVector3d(-0.4,0.1,0);
    s2.s_c = RowVector3d(0.3,0.2,0);

    for(int i=0;i<R.rows();i++)
    {
        for(int j=0;j<R.cols();j++)
        {
            // viewing ray
            Vector3d ray_o = o + double(i)*x_disp + double(j)*y_disp;
            Vector3d convpt(1,1,2);
            Vector3d ray_dir = ray_o - convpt;
            //intersection
            Vector3d S_intersect1 = S_intersection(s1.s_r,s1.s_c,ray_o,ray_dir);
            Vector3d S_intersect2 = S_intersection(s2.s_r,s2.s_c,ray_o,ray_dir);
            const double p = 1000; //phong coeff
            //check intersection
            if(S_intersect1 != s1.s_c)
            {
                Vector3d ray_n = (s1.s_c - S_intersect1).normalized(); //ray normal
                //diffuse shading
                R(i,j) = (S_intersect1 - light1).normalized().dot(ray_n);
                R(i,j) = 0.5*max(R(i,j),0.0);
                R(i,j) += 0.5*max((S_intersect1 - light2).normalized().dot(ray_n),0.);
                //R(i,j) = max(R(i,j),0.0);
                //phong shading
                Vector3d v = S_intersect1 - ray_o;
                Vector3d l1 = S_intersect1 - light1;
                Vector3d l2 = S_intersect1 - light2;
                Vector3d h1 = (v + l1).normalized(); //half vector 1
                Vector3d h2 = (v + l2).normalized(); //half vector 2
                double spec1 = (ray_n).normalized().dot(h1);
                double spec2 = (ray_n).normalized().dot(h2);
                spec1 = max(spec1,0.0);
                spec1 = pow(spec1,p);
                spec2 = max(spec2,0.0);
                spec2 = pow(spec2,p);
                R(i,j) += spec1;
                R(i,j) += spec2;
                G(i,j) = B(i,j) = R(i,j);
                //sphere color=red
                R(i,j) = max(min(R(i,j),1.),0.);
                G(i,j) = max(min(G(i,j),0.),0.);
                B(i,j) = max(min(B(i,j),0.),0.);
                A(i,j) = 1;
            }

            if(S_intersect2 != s2.s_c)
            {
                Vector3d ray_n = (s2.s_c - S_intersect2).normalized(); //ray normal
                //diffuse shading
                R(i,j) = (S_intersect2 - light2).normalized().dot(ray_n); 
                R(i,j) = 0.5*max(R(i,j),0.0);
                R(i,j) += (S_intersect2 - light2).normalized().dot(ray_n);
                R(i,j) = 0.5*max(R(i,j),0.0);
                //phong shading
                Vector3d h1 = (light1 + s2.s_c - ray_o).normalized(); //half vector 1
                Vector3d h2 = (light2 + s2.s_c - ray_o).normalized(); //half vector 2
                double spec1 = (ray_n).normalized().dot(h1);
                double spec2 = (ray_n).normalized().dot(h2);
                spec1 = max(spec1,0.0);
                spec1 = pow(spec1,p);
                spec2 = max(spec2,0.0);
                spec2 = pow(spec2,p);
                R(i,j) += spec1;
                R(i,j) += spec2;
                R(i,j) += 0.3;
                G(i,j) = B(i,j) = R(i,j);
                //sphere color=blue
                R(i,j) = max(min(R(i,j),0.),0.);
                G(i,j) = max(min(G(i,j),0.),0.);
                B(i,j) = max(min(B(i,j),1.),0.);
                A(i,j) = 1;
            }
        }
    }
    // Save to png
    write_matrix_to_png(R,G,B,A,filename);
}

void ray_tracing_mesh_bunny()
{
    cout<<"Ray tracing of triangular meshes for bunny"<<endl;
    const string filename1("bunny.png");
    MatrixXf V2 = MatrixXf::Zero(502,3);
    MatrixXi F2 = MatrixXi::Zero(1000,3);
    ifstream file2;
    file2.open("../data/bunny.off");
    if(file2.is_open())
    {
        int line = 1; //line number
        while(!file2.eof())
        {
            string s;
            getline(file2,s);
            istringstream ss(s); //split with whitespace delimiter
            //fill the V matrix
            if(line != 1 && line != 2 && line<=504)
            {
                int itr=0;
                //cout<<line<<" ";
                do{
                    string num;
                    ss>>num;
                    V2(line - 3,itr) = stof(num); //string to float
                    //cout<<V2(line - 3,itr)<<" ";
                    itr++;
                }while(ss && itr!=3);
            }
            //fill the F matrix
            if(line != 1 && line != 2 && line>504 && line<=1504)
            {
                int itr=0;
                //cout<<line<<" ";
                do{
                    string num;
                    ss>>num;
                    if(itr>0)
                    {
                        F2(line-505,itr-1) = stoi(num); //string to int
                        //cout<<F2(line-505,itr-1)<<" ";
                    }
                    itr++;
                }while(ss && itr!=4);
            }
            //cout<<endl;
            line++;
        }
    }
    file2.close();

    MatrixXd R = MatrixXd::Zero(100,100); // Store the color red
    MatrixXd G = MatrixXd::Zero(100,100); // Store the color green
    MatrixXd B = MatrixXd::Zero(100,100); // Store the color blue
    MatrixXd A = MatrixXd::Zero(100,100); // Store the alpha mask
    //Camera 
    Vector3d o(-0.2,0.2,1);
    Vector3d x_disp(0.4/R.cols(),0,0);
    Vector3d y_disp(0,-0.4/R.rows(),0);

    //light source
    const Vector3d light1(0.3,-0.2,-2);

    for(int i=0;i<R.rows();i++)
    {
        for(int j=0;j<R.cols();j++)
        {
            // viewing ray
            Vector3d ray_o = o + double(i)*x_disp + double(j)*y_disp;
            Vector3d ray_dir = RowVector3d(0,0,-1);

            //trace bunny
            for(int m=0;m<F2.rows();m++)
            {
                
               Vector3d a(V2(F2(m,0),0),V2(F2(m,0),1),V2(F2(m,0),2));            
               Vector3d b(V2(F2(m,1),0),V2(F2(m,1),1),V2(F2(m,1),2));
               Vector3d c(V2(F2(m,2),0),V2(F2(m,2),1),V2(F2(m,2),2));

               Vector3d T_intersect = T_intersection(ray_o,ray_dir,a,b,c);

                //check intersection
                if(T_intersect(0) > 0)
                {
                    //R(i,j) = 1;
                    Vector3d n = ((b-a).cross(c-a)).normalized();
                    Vector3d inter = ray_o + T_intersect(0)*ray_dir;
                    R(i,j) = max((0.5*(light1 - inter)).normalized().dot(n),0.); // diffuse shading
                    //red
                    A(i,j) = 1;
                }
            }
        }
    }
    // Save to png
    write_matrix_to_png(R,G,B,A,filename1);
}

void ray_tracing_mesh_bumpycube()
{
    cout<<"Ray tracing of bumpy cube mesh"<<endl;
    const string filename1("bumpycube.png");
    MatrixXf V1 = MatrixXf::Zero(502,3);
    MatrixXi F1 = MatrixXi::Zero(1000,3);
    ifstream file1;
    file1.open("../data/bumpy_cube.off");
    if(file1.is_open())
    {
        int line = 1; //line number
        while(!file1.eof())
        {
            string s;
            getline(file1,s);
            istringstream ss(s); //split with whitespace delimiter
            //fill the V matrix
            if(line != 1 && line != 2 && line<=504)
            {
                int itr=0;
                //cout<<line<<" ";
                do{
                    string num;
                    ss>>num;
                    V1(line - 3,itr) = stof(num); //string to float
                    //cout<<V1(line - 3,itr)<<" ";
                    itr++;
                }while(ss && itr!=3);
            }
            //fill the F matrix
            if(line != 1 && line != 2 && line>504 && line<=1504)
            {
                int itr=0;
                //cout<<line<<" ";
                do{
                    string num;
                    ss>>num;
                    if(itr>0)
                    {
                        F1(line-505,itr-1) = stoi(num); //string to int
                        //cout<<F1(line-505,itr-1)<<" ";
                    }
                    itr++;
                }while(ss && itr!=4);
            }
            //cout<<endl;
            line++;
        }
    }
    file1.close();
    //trace bumpy cube

    MatrixXd R = MatrixXd::Zero(50,50); // Store the color red
    MatrixXd G = MatrixXd::Zero(50,50); // Store the color green
    MatrixXd B = MatrixXd::Zero(50,50); // Store the color blue
    MatrixXd A = MatrixXd::Zero(50,50); // Store the alpha mask
    //Camera 
    Vector3d o(-5,5,1);
    Vector3d x_disp(10.0/R.cols(),0,0);
    Vector3d y_disp(0,-10.0/R.rows(),0);

    //light source
    const Vector3d light1(0.3,-0.2,-2);
    
    for(int i=0;i<R.rows();i++)
    {
        for(int j=0;j<R.cols();j++)
        {
            // viewing ray
            Vector3d ray_o = o + double(i)*x_disp + double(j)*y_disp;
            Vector3d ray_dir = RowVector3d(0,0,-1);

            //trace bumpy cube
            for(int m=0;m<F1.rows();m++)
            {
               Vector3d a(V1(F1(m,0),0),V1(F1(m,0),1),V1(F1(m,0),2));            
               Vector3d b(V1(F1(m,1),0),V1(F1(m,1),1),V1(F1(m,1),2));
               Vector3d c(V1(F1(m,2),0),V1(F1(m,2),1),V1(F1(m,2),2));

               Vector3d T_intersect = T_intersection(ray_o,ray_dir,a,b,c);

                //check intersection
                if(T_intersect(0) > 0)
                {
                    //R(i,j) = 1;
                    Vector3d n = ((b-a).cross(c-a)).normalized();
                    Vector3d inter = ray_o + T_intersect(0)*ray_dir;
                    //Vector3d inter = ray_o - a + T_intersect(1)*(b-a) + T_intersect(2)*(c-a);
                    B(i,j) = 0.5*max((light1 - inter).normalized().dot(n),0.); // diffuse shading
                    //blue
                    A(i,j) = 1;
                }
            }
        }
    }
    // Save to png
    write_matrix_to_png(R,G,B,A,filename1);
}
void shadows()
{
    cout<<"Shadows for a sphere"<<endl;
    const string filename("shadows1.png");
    MatrixXd R = MatrixXd::Constant(800,800,0.0); // Store the color red
    MatrixXd G = MatrixXd::Constant(800,800,0.0); // Store the color green
    MatrixXd B = MatrixXd::Constant(800,800,0.0); // Store the color blue
    MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

    //Camera 
    Vector3d o(-1,1,1);
    Vector3d x_disp(2.0/R.cols(),0,0);
    Vector3d y_disp(0,-2.0/R.rows(),0);

    // Single light source
    const Vector3d light1(0.3,2,0);

    Sphere s1;
    s1.s_r = 0.4;
    s1.s_c = RowVector3d(1,0.1,0);

    //triangle 1
    Vector3d a(-100,1,-100);
    Vector3d b(-100,-1,100);
    Vector3d c(100,-1,100);
    //triangle 2
    Vector3d d(-100,-1,-100);
    Vector3d e(100,-1,-100);
    Vector3d f(100,-1,100);
    for(int i=0;i<R.rows();i++)
    {
        for(int j=0;j<R.cols();j++)
        {
            // viewing rayperspective
            Vector3d ray_o = o + double(i)*x_disp + double(j)*y_disp;
            Vector3d ray_dir = (ray_o - Vector3d(-1,0,2)).normalized();
            //intersection
            Vector3d S_intersect1 = S_intersection(s1.s_r,s1.s_c,ray_o,ray_dir);
            Vector3d T_intersect1 = T_intersection(ray_o,ray_dir,a,b,c);
            Vector3d T_intersect2 = T_intersection(ray_o,ray_dir,d,e,f);
            //plane
            if(T_intersect1(0) > 0 || T_intersect2(0) > 0)
            {
                double t = T_intersect1(0)>T_intersect2(0)?T_intersect1(0):T_intersect2(0);
                Vector3d intergrd = ray_o + t*ray_dir;
                Vector3d shad_ray = light1 - intergrd;
                Vector3d S_inter = S_intersection(s1.s_r,s1.s_c,intergrd,shad_ray);
                if(S_inter != s1.s_c)
                {
                    R(i,j) = R(i,j)*0.35;
                    G(i,j) = G(i,j)*0.35;
                    B(i,j) = B(i,j)*0.35;
                }
                else{
                    R(i,j) = 0;
                    B(i,j) = 0.6;
                    G(i,j) = 0.6;
                }
                A(i,j) = 1;
            }
            //check intersection
            if(S_intersect1 != s1.s_c)
            {
                Vector3d ray_n = (s1.s_c - S_intersect1).normalized(); //ray normal
                //diffuse shading
                R(i,j) = (S_intersect1 - light1).normalized().dot(ray_n);
                R(i,j) = max(R(i,j),0.0);
                G(i,j) = B(i,j) = R(i,j);
                //sphere color=red
                R(i,j) = min(R(i,j),1.);
                G(i,j) = min(G(i,j),0.);
                B(i,j) = min(B(i,j),0.);
                A(i,j) = 1;
            }
        }
    }
    // Save to png
    write_matrix_to_png(R,G,B,A,filename);
}

void reflections()
{
    cout<<"Reflection for all the objects"<<endl;
    const string filename("reflect1.png");
    MatrixXd R = MatrixXd::Constant(800,800,0.0); // Store the color red
    MatrixXd G = MatrixXd::Constant(800,800,0.0); // Store the color green
    MatrixXd B = MatrixXd::Constant(800,800,0.0); // Store the color blue
    MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

    //Camera 
    Vector3d o(-1,1,1);
    Vector3d x_disp(2.0/R.cols(),0,0);
    Vector3d y_disp(0,-2.0/R.rows(),0);

    // Single light source
    const Vector3d light1(0.3,2,0);

    Sphere s1;
    s1.s_r = 0.4;
    s1.s_c = RowVector3d(1,0.1,0);

    //triangle 1
    Vector3d a(-100,1,-100);
    Vector3d b(-100,-1,100);
    Vector3d c(100,-1,100);
    //triangle 2
    Vector3d d(-100,-1,-100);
    Vector3d e(100,-1,-100);
    Vector3d f(100,-1,100);
    for(int i=0;i<R.rows();i++)
    {
        for(int j=0;j<R.cols();j++)
        {
            // viewing rayperspective
            Vector3d ray_o = o + double(i)*x_disp + double(j)*y_disp;
            Vector3d ray_dir = (ray_o - Vector3d(-1,0,2)).normalized();
            //intersection
            Vector3d S_intersect1 = S_intersection(s1.s_r,s1.s_c,ray_o,ray_dir);
            Vector3d T_intersect1 = T_intersection(ray_o,ray_dir,a,b,c);
            Vector3d T_intersect2 = T_intersection(ray_o,ray_dir,d,e,f);
            //plane
            if(T_intersect1(0) > 0 || T_intersect2(0) > 0)
            {
                double t = T_intersect1(0)>T_intersect2(0)?T_intersect1(0):T_intersect2(0);
                Vector3d intergrd = ray_o + t*ray_dir;
                Vector3d n = intergrd - Vector3d(intergrd(0),6,intergrd(2));
                Vector3d ref_ray = (ray_dir - (2*ray_dir.dot(n))*n).normalized();
                Vector3d S_inter = S_intersection(s1.s_r,s1.s_c,intergrd,ref_ray);
                if(S_inter != s1.s_c)
                {
                    Vector3d ray_n = (s1.s_c - S_inter).normalized(); //ray normal
                    //diffuse shading
                    R(i,j) = (S_inter - light1).normalized().dot(ray_n);
                    R(i,j) = max(R(i,j),0.0);
                    A(i,j) = 1;}
                else{
                    R(i,j) = 1;
                    B(i,j) = 1;
                    G(i,j) = 1;
                    A(i,j) = 1;
            }
            }
            //check intersection
            if(S_intersect1 != s1.s_c)
            {
                Vector3d ray_n = (s1.s_c - S_intersect1).normalized(); //ray normal
                //diffuse shading
                R(i,j) = (S_intersect1 - light1).normalized().dot(ray_n);
                R(i,j) = max(R(i,j),0.0);
                G(i,j) = B(i,j) = R(i,j);
                //sphere color=red
                R(i,j) = min(R(i,j),1.);
                G(i,j) = min(G(i,j),0.);
                B(i,j) = min(B(i,j),0.);
                A(i,j) = 1;
            }
            
        }
    }
    // Save to png
    write_matrix_to_png(R,G,B,A,filename);
}
int main()
{
    ray_tracing_spheres();
    shading();
    perspective_proj();
    ray_tracing_mesh_bunny();
    ray_tracing_mesh_bumpycube();
    shadows();
    reflections();
    return 0;
}
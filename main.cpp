#include <iostream>
#include <vector>
#include <sstream>
#include "tinyxml2/tinyxml2.h"

using namespace std;
using namespace tinyxml2;

void print_usage() {
    cout << "Usage: inertia_calc [urdf file]" << endl;
}

template<class T>
void split(const std::string &s, char delim, T result) {
    stringstream ss(s); string item;
    while (getline(ss, item, delim)) {
        *(result++) = item;
    }
}

vector<string> split(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, back_inserter(elems));
    return elems;
}

void calculate_link_inertia_mx(double m, XMLElement* geom, XMLElement* inert) {
    const char* s1;
    double L, R;
    double sX, sY, sZ;
    double ixx, ixy, ixz;
    double      iyy, iyz;
    double           izz;
    if (strcmp(geom->Name(), "box") == 0) {
        // Get size
        geom->QueryStringAttribute("size", &s1);
        cout << "Box size is " << s1 << endl;
        vector<string> sizes = split(string(s1),' ');
        sX = stod(sizes[0]);
        sY = stod(sizes[1]);
        sZ = stod(sizes[2]);
        ixx = (1.0/12.0)*m*(sY*sY + sZ*sZ); // Ix = 1/12 * m (sY^2 + sZ^2)
        iyy = (1.0/12.0)*m*(sX*sX + sZ*sZ);
        izz = (1.0/12.0)*m*(sX*sX + sY*sY);
        ixy = ixz = iyz = 0;
    } else if (strcmp(geom->Name(), "cylinder") == 0) {
        // Get length and radius
        geom->QueryDoubleAttribute("length", &L);
        geom->QueryDoubleAttribute("radius", &R);
        cout << "Cylinder L = " << L << ", R = " << R << endl;
        // Axis is Z. TODO: Is this true?
        izz = 0.5*m*R*R; // 1/2 m R^2
        ixx = iyy = 0.25*m*R*R + (1.0/12.0)*m*L*L; // 1/4 m R^2 + 1/12 m H^2
        ixy = ixz = iyz = 0;
    } else if (strcmp(geom->Name(), "sphere") == 0) {
        geom->QueryDoubleAttribute("radius", &R);
        cout << "Sphere R = " << R << endl;
        ixx = iyy = izz = (2.0/5.0)*m*R*R; // Solid sphere
        ixy = ixz = iyz = 0;
    }
    inert->SetAttribute("ixx", ixx);
    inert->SetAttribute("iyy", iyy);
    inert->SetAttribute("izz", izz);
    inert->SetAttribute("ixy", ixy);
    inert->SetAttribute("ixz", ixz);
    inert->SetAttribute("iyz", iyz);
}

bool update_link_inertia_mx(XMLElement* link) {
    const char* linkname;
    link->QueryStringAttribute("name", &linkname);
    XMLElement *e;
    XMLElement *geom;
    XMLElement *inert;
    // Get geometry
    e = link->FirstChildElement("collision");
    if (e == nullptr) {
        cout << "Link: '" << linkname << "' has no collision info." << endl;
        return false;
    } else {
        e = e->FirstChildElement("geometry");
        if (e == nullptr) {
            cout << "Link: '" << linkname << "' has no collision::geometry info." << endl;
            return false;
        } else {
            geom = e->FirstChildElement();
            if (geom == nullptr) {
                cout << "Link: '" << linkname << "' has no collision::geometry info." << endl;
                return false;
            } else if (strcmp(geom->Name(), "box") == 0) {
                cout << "Link: '" << linkname << "' has collision::geometry::box info." << endl;
            } else if (strcmp(geom->Name(), "cylinder") == 0) {
                cout << "Link: '" << linkname << "' has collision::geometry::cylinder info." << endl;
            } else if (strcmp(geom->Name(), "sphere") == 0) {
                cout << "Link: '" << linkname << "' has collision::geometry::sphere info." << endl;
            } else if (strcmp(geom->Name(), "mesh") == 0) {
                cout << "Link: '" << linkname << "' has collision::geometry::mesh info, but that inertia source is not supported!" << endl;
                return false;
            }
        }
    }
    // Get mass from 'inertial'
    e = link->FirstChildElement("inertial");
    double mass;
    if (e == nullptr) {
        cout << "Link: '" << linkname << "' has no inertial info." << endl;
        return false;
    } else {
        inert = e->FirstChildElement("inertia");
        e = e->FirstChildElement("mass");
        if (e == nullptr) {
            cout << "Link: '" << linkname << "' has no inertial::mass info." << endl;
            return false;
        } else {
            e->QueryDoubleAttribute("value", &mass);
            cout << "Link: '" << linkname << "' has mass = " << mass << endl;
        }
        if (inert == nullptr) { // TODO: Could insert a new node instead of error!
            cout << "Link: '" << linkname << "' has no inertial::inertia info." << endl;
            return false;
        } else {
            calculate_link_inertia_mx(mass, geom, inert);
        }
    }
    return true;
}

void inertia_calc_urdf(const char* path_to_urdf) {
    cout << "Loading URDF: " << path_to_urdf << endl;
    XMLDocument doc;
    doc.LoadFile(path_to_urdf);
    XMLNode *root = doc.FirstChildElement("robot");
    if (root == nullptr) {
        cout << "Error: malformed URDF XML!" << endl; return;
    }
    XMLElement *link = root->FirstChildElement("link");
    while (link != nullptr) {
        if (update_link_inertia_mx(link)) {
            cout << "Mass moment of inertia matrix has been updated!" << endl;
        }
        link = link->NextSiblingElement("link");
    }
    string output = string(path_to_urdf);
    output += ".out.urdf";
    doc.SaveFile(output.c_str());
}

int main(int argc, char* argv[]) {
    cout << "ROS::URDF inertia matrix calculator\n";
    if (argc < 2) {
        print_usage();
    } else {
        inertia_calc_urdf(argv[1]);
    }
    return 0;
}
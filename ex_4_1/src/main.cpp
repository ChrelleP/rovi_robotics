#include <iostream>
#include <vector>
#include <rw/math.hpp>

rw::math::Transform3D<> forwardKinematics(std::vector<rw::math::Transform3D<>> const trefs,
                                          unsigned int const idx, rw::math::Q const q) {
    if(trefs.size() != q.size()) {
        RW_THROW("The number of local transformations must be equal to the length of the configuration vector");
    }

    rw::math::Transform3D<> base_Ti = trefs[0];
    for(unsigned int i = 1; i < idx; ++i) {
        rw::math::Transform3D<> Rz(rw::math::RPY<>(q[i],0,0).toRotation3D());
        base_Ti = base_Ti*trefs[i]*Rz;
    }

    return base_Ti;
}

std::vector<rw::math::Transform3D<>> generate_trefs() {
  // Create Tref vector
  std::vector<rw::math::Transform3D<>> Trefs;

  // Joint base -> 1:
  rw::math::Vector3D<> V1(0, 0, 3);
  rw::math::RPY<> R1(0, 0, 0);
  rw::math::Transform3D<> T1(V1, R1.toRotation3D());
  Trefs.push_back(T1);

  // Joint 1 -> 2:
  rw::math::Vector3D<> V2(0, 0, 0);
  rw::math::RPY<> R2(0, 0, rw::math::Pi/2);
  rw::math::Transform3D<> T2(V2, R2.toRotation3D());
  Trefs.push_back(T2);

  // Joint 2 -> 3:
  rw::math::Vector3D<> V3(0, 2, 0);
  rw::math::RPY<> R3(0, 0, 0);
  rw::math::Transform3D<> T3(V3, R3.toRotation3D());
  Trefs.push_back(T3);

  return Trefs;
}


int main() {

    // Joint 3 -> TCP:
    rw::math::Vector3D<> VTCP(2, 0, 0);
    rw::math::RPY<> RTCP(0, 0, 0);
    rw::math::Transform3D<> TTCP(VTCP, RTCP.toRotation3D());

    std::vector<rw::math::Transform3D<>> Trefs = generate_trefs();

    rw::math::Jacobian Jacobian(3);

    rw::math::Q q(3, 0.0, -rw::math::Pi/6, rw::math::Pi/6);

    rw::math::Vector3D<> b;
    rw::math::Vector3D<> a;

    auto base_TCP = forwardKinematics(Trefs,3,q)*TTCP;

    for (int i = 1; i < 4; i++) {
      auto base_i = forwardKinematics(Trefs, i, q);

      b = base_i.R().getCol(2);               // THESE NEEDS TO BE ADDED
      a = cross(b,(base_TCP.P()-base_i.P()));

      rw::math::Vector3D<> rtr(99, 99, 99); // TESTER

      Jacobian.addRotation(rtr,0,i-1); // TODO REPACE NOT ADD
      Jacobian.addPosition(a,i-1,0);


      std::cout << "B_base_"<< i << ": " << b << std::endl;
      std::cout << "A_base_"<< i << ": " << a << std::endl;
    }

    std::cout<<"\nJACOBIAN: \n\n"<<Jacobian<<std::endl;

    return 0;
}

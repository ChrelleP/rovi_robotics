#include <iostream>
#include <vector>
#include <rw/math.hpp>

rw::math::Transform3D<> forwardKinematics(std::vector<rw::math::Transform3D<>> const trefs,
                                          unsigned int const idx, rw::math::Q const q) {
    if(trefs.size() != q.size()) {
        RW_THROW("The number of local transformations must be equal to the length of the configuration vector");
    }

    rw::math::Transform3D<> baseTi;

    for(unsigned int i = 0; i < idx; ++i) {
        rw::math::Transform3D<> Rz(rw::math::RPY<>(q[i],0,0).toRotation3D());
        baseTi = baseTi*trefs[i]*Rz;
    }

    return baseTi;
}

int main() {
    // Create Tref vector
    std::vector<rw::math::Transform3D<>> Trefs;
    // Joint 0: <RPY> 0 0 0 </RPY> <Pos> 0 0 0 </Pos>
    rw::math::Vector3D<> V0(0, 0, 0);
    rw::math::RPY<> R0(0, 0, 0);
    rw::math::Transform3D<> T0(V0, R0.toRotation3D());
    Trefs.push_back(T0);

    // Joint 1: <RPY> 90 0 90 </RPY> <Pos> 0 0 0.08920 </Pos>
    rw::math::Vector3D<> V1(0, 0, 0.08920);
    rw::math::RPY<> R1(90*rw::math::Deg2Rad, 0, 90*rw::math::Deg2Rad);
    rw::math::Transform3D<> T1(V1, R1.toRotation3D());
    Trefs.push_back(T1);

    // Joint 2: <RPY> 270 0 0 </RPY> <Pos> 0 0.425 0 </Pos>
    rw::math::Vector3D<> V2(0, 0.425, 0);
    rw::math::RPY<> R2(270*rw::math::Deg2Rad, 0, 0);
    rw::math::Transform3D<> T2(V2, R2.toRotation3D());
    Trefs.push_back(T2);

    // Joint 3: <RPY> 0 0 0 </RPY> <Pos> -0.39243 0 0 </Pos>
    rw::math::Vector3D<> V3(-0.39243, 0, 0);
    rw::math::RPY<> R3(0, 0, 0);
    rw::math::Transform3D<> T3(V3, R3.toRotation3D());
    Trefs.push_back(T3);

    // Joint 4: <RPY> 270 0 90 </RPY> <Pos> 0 0 0.109 </Pos>
    rw::math::Vector3D<> V4(0, 0, 0.109);
    rw::math::RPY<> R4(270*rw::math::Deg2Rad, 0, 90*rw::math::Deg2Rad);
    rw::math::Transform3D<> T4(V4, R4.toRotation3D());
    Trefs.push_back(T4);

    // Joint 5: <RPY> 0 0 270 </RPY> <Pos> 0 0 0.093 </Pos>
    rw::math::Vector3D<> V5(0, 0, 0.093);
    rw::math::RPY<> R5(0, 0, 270*rw::math::Deg2Rad);
    rw::math::Transform3D<> T5(V5, R5.toRotation3D());
    Trefs.push_back(T5);

    // TCP: <RPY> 270 0 0 </RPY> <Pos> 0 0 0.082 </Pos>
    rw::math::Vector3D<> VTCP(0, 0, 0.082);
    rw::math::RPY<> RTCP(270*rw::math::Deg2Rad, 0, 0);
    rw::math::Transform3D<> TTCP(VTCP, RTCP.toRotation3D());

    rw::math::Q q(6, 0.859, 0.208, -0.825, -0.746,  -1.632, 1.527);
    auto baseTtcp = forwardKinematics(Trefs, 6, q)*TTCP;

    std::cout << baseTtcp.P() << " " << rw::math::RPY<>(baseTtcp.R()) << std::endl;
    return 0;
}

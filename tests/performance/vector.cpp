// Copyright (C) 2022 Nikunj Gupta
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
//  Software Foundation, version 3.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along
// with this program. If not, see <https://www.gnu.org/licenses/>.

#include "base.decl.h"

#include <eigen3/Eigen/Core>

class Main : public CBase_Main
{
public:
    Main(CkArgMsg* msg)
    {
        int total_size = atoi(msg->argv[1]);
        int decomposition_factor = atoi(msg->argv[2]);
        int min_size = atoi(msg->argv[3]);

        int block_size = std::max(
            min_size, total_size / (CkNumPes() * decomposition_factor));

        int num_chares = total_size / block_size;

        ckout << "TOTAL SIZE: " << total_size
              << ", DECOMPOSITION FACTOR: " << decomposition_factor
              << ", MIN SIZE: " << min_size << endl
              << "TOTAL CHARES CREATED: " << num_chares
              << ", BLOCK SIZE: " << block_size << endl;

        start_time = CkWallTimer();
        CProxy_vector proxy =
            CProxy_vector::ckNew(block_size, thisProxy, num_chares);
        proxy.compute();
    }

    void finish()
    {
        double end_time = CkWallTimer();

        ckout << "EXECUTION TIME: " << end_time - start_time << endl;
        CkExit();
    }

private:
    double start_time;
};

class vector : public CBase_vector
{
public:
    vector(int block_size, CProxy_Main proxy)
    {
        vect.push_back(Eigen::VectorXd::Constant(block_size, 0.));
        vect.push_back(Eigen::VectorXd::Constant(block_size, 1.));
        vect.push_back(Eigen::VectorXd::Constant(block_size, 1.));
        vect.push_back(Eigen::VectorXd::Constant(block_size, 1.));
        vect.push_back(Eigen::VectorXd::Constant(block_size, 1.));
        proxy_ = proxy;
    }

    void compute()
    {
        vect[0] = vect[1] + vect[2] + vect[3] + vect[4];

        CkCallback cb(CkReductionTarget(Main, finish), proxy_);
        contribute(cb);
    }

private:
    std::vector<Eigen::VectorXd> vect;
    CProxy_Main proxy_;
};

#include "base.def.h"
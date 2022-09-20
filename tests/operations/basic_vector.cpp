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

#include <charmtyles/charmtyles.hpp>

#include "base.decl.h"

class Main : public CBase_Main
{
public:
    Main(CkArgMsg* msg)
    {
        int num_pes = 6;
        if (msg->argc > 1)
            num_pes = atoi(msg->argv[1]);

        ct::init(num_pes);

        thisProxy.benchmark();
    }

    void benchmark()
    {
        constexpr std::size_t VEC_SIZE = 10000000;

        double start = CkWallTimer();

        ct::vector A{VEC_SIZE, 1.};
        ct::vector B{VEC_SIZE, 2.};
        ct::vector C{VEC_SIZE, -4.};
        ct::vector D{VEC_SIZE, 3.};

        ct::vector E = A + B + C + D;

        // ct::frontend::ASTQueue& queue =
        //     CT_ACCESS_SINGLETON(ct::frontend::ast_queue);
        // queue.print_instructions();

        ct::sync();

        double end = CkWallTimer();

        ckout << "Execution Time: " << end - start << endl;

        CkExit();
    }
};

#include "base.def.h"
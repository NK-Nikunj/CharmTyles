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

#pragma once

#include <charmtyles/util/AST.hpp>
#include <charmtyles/util/view_vector.hpp>

#include <charmtyles/frontend/matrix.hpp>
#include <charmtyles/frontend/operators.hpp>
#include <charmtyles/frontend/vector.hpp>

#include <charmtyles/backend/charmtyles_base.hpp>

namespace ct {

    void init(std::size_t num_pes)
    {
        CProxy_set_future set_future_proxy = CProxy_set_future::ckNew();
        CProxy_charmtyles_base proxy =
            CProxy_charmtyles_base::ckNew(num_pes, set_future_proxy, num_pes);
        ct_proxy = proxy;
        sf_proxy = set_future_proxy;
    }

    void sync()
    {
        using AST = std::vector<ct::frontend::ASTNode>;

        ct::frontend::ASTQueue& queue =
            CT_ACCESS_SINGLETON(ct::frontend::ast_queue);

        int sdag_index = queue.sdag_index();
        std::vector<AST> instr_list = queue.dispatch();
        ck::future<bool> is_done;

        ct_proxy.compute(sdag_index, instr_list);
        sf_proxy.put_future(is_done);

        is_done.get();

        return;
    }
}    // namespace ct

#include <charmtyles/backend/libcharmtyles.def.h>
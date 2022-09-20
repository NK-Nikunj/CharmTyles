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

#include <charmtyles/frontend/vector.hpp>
#include <charmtyles/util/AST.hpp>

namespace ct {

    template <typename LHS, typename RHS,
        typename = typename ct::traits::isVecType<LHS, RHS>::type>
    VecExpression<LHS, RHS> operator+(LHS const& lhs, RHS const& rhs)
    {
        return VecExpression<LHS, RHS>(
            lhs, rhs, lhs.size(), ct::frontend::Operation::add);
    }

    template <typename LHS, typename RHS,
        typename = typename ct::traits::isVecType<LHS, RHS>::type>
    VecExpression<LHS, RHS> operator-(LHS const& lhs, RHS const& rhs)
    {
        return VecExpression<LHS, RHS>(
            lhs, rhs, lhs.size(), ct::frontend::Operation::sub);
    }

}    // namespace ct

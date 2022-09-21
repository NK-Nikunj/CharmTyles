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

#include <charmtyles/frontend/matrix.hpp>
#include <charmtyles/frontend/vector.hpp>

namespace ct {

    namespace traits {

        template <typename... Ts>
        struct isArrayTypeImpl<ct::Expression<Ts...>>
        {
            using type = void;
        };

        template <typename LHS, typename RHS>
        struct isArrayType
        {
            using type_LHS =
                typename isArrayTypeImpl<typename std::decay<LHS>::type>::type;
            using type_RHS =
                typename isArrayTypeImpl<typename std::decay<RHS>::type>::type;

            using type = std::enable_if_t<std::is_same_v<type_LHS, type_RHS>>;
        };
    }    // namespace traits

    template <typename LHS, typename RHS,
        typename = typename ct::traits::isArrayType<LHS, RHS>::type>
    Expression<LHS, RHS> operator+(LHS const& lhs, RHS const& rhs)
    {
        return Expression<LHS, RHS>(
            lhs, rhs, lhs.size(), ct::frontend::Operation::add);
    }

    template <typename LHS, typename RHS,
        typename = typename ct::traits::isArrayType<LHS, RHS>::type>
    Expression<LHS, RHS> operator-(LHS const& lhs, RHS const& rhs)
    {
        return Expression<LHS, RHS>(
            lhs, rhs, lhs.size(), ct::frontend::Operation::sub);
    }

}    // namespace ct

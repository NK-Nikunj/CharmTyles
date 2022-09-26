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

#include <cassert>
#include <cstdint>
#include <type_traits>

#include <charmtyles/util/AST.hpp>

namespace ct {

    // User Facing ct::scalar class
    class scalar
    {
        template <typename LHS, typename RHS>
        friend class Expression;

    public:
        scalar()
          : id_(ct::frontend::get_scalar_id())
          , node_(id_, ct::frontend::Operation::init_random,
                ct::frontend::Type::scalar)
        {
            ct::frontend::ASTQueue& queue =
                CT_ACCESS_SINGLETON(ct::frontend::ast_queue);

            queue.insert(node_);
        }

        explicit scalar(double value)
          : id_(ct::frontend::get_scalar_id())
          , value_(value)
          , node_(id_, ct::frontend::Operation::init_value, value,
                ct::frontend::Type::scalar, value)
        {
            ct::frontend::ASTQueue& queue =
                CT_ACCESS_SINGLETON(ct::frontend::ast_queue);

            queue.insert(node_);
        }

        scalar(scalar const& other)
          : id_(ct::frontend::get_scalar_id())
          , value_(other.value_)
          , node_(id_, ct::frontend::Operation::copy, other.node_)
        {
        }

        scalar(scalar&& other) = delete;

        template <typename Expression>
        scalar(Expression const& e)
          : id_(ct::frontend::get_scalar_id())
        {
            std::vector<ct::frontend::ASTNode> instr = e();
            ct::frontend::ASTNode& root = instr.front();

            root.name_ = id_;
            node_ = ct::frontend::ASTNode{root};
            value_ = root.value_;

            ct::frontend::ASTQueue& queue =
                CT_ACCESS_SINGLETON(ct::frontend::ast_queue);
            queue.insert(instr);
        }

        template <typename Expression>
        scalar& operator=(Expression const& e)
        {
            std::vector<ct::frontend::ASTNode> instr = e();
            ct::frontend::ASTNode& root = instr.front();

            root.name_ = id_;

            ct::frontend::ASTQueue& queue =
                CT_ACCESS_SINGLETON(ct::frontend::ast_queue);
            queue.insert(instr);

            return *this;
        }

        scalar& operator=(scalar const& other)
        {
            ct::frontend::ASTNode node(
                id_, ct::frontend::Operation::copy, other.node_);

            ct::frontend::ASTQueue& queue =
                CT_ACCESS_SINGLETON(ct::frontend::ast_queue);

            queue.insert(node);

            return *this;
        }

        void pup(PUP::er& p)
        {
            p | id_;
            p | value_;
            p | node_;
        }

        // Return the last updated value
        double value() const
        {
            return value_;
        }

    private:
        std::vector<ct::frontend::ASTNode> operator()() const
        {
            ct::frontend::ASTNode new_node{node_};
            new_node.operation_ = ct::frontend::Operation::noop;
            return std::vector<ct::frontend::ASTNode>{new_node};
        }

        // ID or Name of the scalar
        std::size_t id_;

        // Last updated value in the scalar
        double value_;

        // AST Node Reference
        ct::frontend::ASTNode node_;
    };

    namespace traits {

        template <>
        struct isArrayTypeImpl<ct::scalar>
        {
            using type = void;
        };

    }    // namespace traits

}    // namespace ct
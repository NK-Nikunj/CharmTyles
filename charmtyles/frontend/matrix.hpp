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

    // User Facing ct::matrix class
    class matrix
    {
        template <typename LHS, typename RHS>
        friend class Expression;

    public:
        matrix() = default;

        explicit matrix(std::size_t rows, std::size_t cols)
          : id_(ct::frontend::get_matrix_id())
          , rows_(rows)
          , cols_(cols)
          , node_(id_, ct::frontend::Operation::init_random,
                ct::frontend::Type::matrix, rows, cols)
        {
            ct::frontend::ASTQueue& queue =
                CT_ACCESS_SINGLETON(ct::frontend::ast_queue);

            queue.insert(node_);
        }

        explicit matrix(std::size_t rows, std::size_t cols, double value)
          : id_(ct::frontend::get_matrix_id())
          , rows_(rows)
          , cols_(cols)
          , node_(id_, ct::frontend::Operation::init_value, value,
                ct::frontend::Type::matrix, rows, cols)
        {
            ct::frontend::ASTQueue& queue =
                CT_ACCESS_SINGLETON(ct::frontend::ast_queue);

            queue.insert(node_);
        }

        matrix(matrix const& other)
          : id_(ct::frontend::get_matrix_id())
          , rows_(other.rows_)
          , cols_(other.cols_)
          , node_(id_, ct::frontend::Operation::copy, other.node_)
        {
        }

        matrix(matrix&& other) = delete;

        template <typename Expression>
        matrix(Expression const& e)
          : id_(ct::frontend::get_matrix_id())
        {
            std::vector<ct::frontend::ASTNode> instr = e();
            ct::frontend::ASTNode& root = instr.front();

            root.name_ = id_;
            node_ = ct::frontend::ASTNode{root};
            rows_ = root.mat_dim_.first;
            cols_ = root.mat_dim_.second;

            ct::frontend::ASTQueue& queue =
                CT_ACCESS_SINGLETON(ct::frontend::ast_queue);
            queue.insert(instr);
        }

        template <typename Expression>
        matrix& operator=(Expression const& e)
        {
            std::vector<ct::frontend::ASTNode> instr = e();
            ct::frontend::ASTNode& root = instr.front();

            root.name_ = id_;

            ct::frontend::ASTQueue& queue =
                CT_ACCESS_SINGLETON(ct::frontend::ast_queue);
            queue.insert(instr);

            return *this;
        }

        matrix& operator=(matrix const& other)
        {
            assert((rows_ == other.rows_ && cols_ == other.cols_) &&
                "Vector Dimensions do not match!");

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
            p | rows_;
            p | cols_;
            p | node_;
        }

        std::size_t rows() const
        {
            return rows_;
        }

        std::size_t cols() const
        {
            return cols_;
        }

    private:
        std::vector<ct::frontend::ASTNode> operator()() const
        {
            ct::frontend::ASTNode new_node{node_};
            new_node.operation_ = ct::frontend::Operation::noop;
            return std::vector<ct::frontend::ASTNode>{new_node};
        }

        // ID or Name of the Vector
        std::size_t id_;

        // Size of the Vector
        std::size_t rows_;
        std::size_t cols_;

        // AST Node Reference
        ct::frontend::ASTNode node_;
    };

    namespace traits {

        template <>
        struct isArrayTypeImpl<ct::matrix>
        {
            using type = void;
        };

    }    // namespace traits

}    // namespace ct
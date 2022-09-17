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

#include <cstdint>

#include <charmtyles/frontend/AST.hpp>

namespace ct {

    template <typename LHS, typename RHS>
    class VecExpression
    {
    public:
        VecExpression(LHS const& lhs_, RHS const& rhs_, std::size_t size,
            ct::frontend::Operation op_)
          : lhs(lhs_)
          , rhs(rhs_)
          , size_(size)
          , op(op_)
        {
        }

        std::vector<ct::frontend::ASTNode> operator()() const
        {
            std::vector<ct::frontend::ASTNode> left = lhs();
            std::vector<ct::frontend::ASTNode> right = rhs();

            ct::frontend::ASTNode node{static_cast<std::size_t>(-1), op,
                ct::frontend::Type::vector, size_};
            node.left_ = 1;
            node.right_ = left.size() + 1;

            std::vector<ct::frontend::ASTNode> ast;
            ast.reserve(left.size() + right.size() + 1);

            ast.emplace_back(node);
            std::copy(left.begin(), left.end(), std::back_inserter(ast));
            std::copy(right.begin(), right.end(), std::back_inserter(ast));

            // Update left and right neighbors
            for (int i = 1; i != left.size(); ++i)
            {
                if (ast[i].left_ != static_cast<std::size_t>(-1))
                {
                    ast[i].left_ += 1;
                }

                if (ast[i].right_ != static_cast<std::size_t>(-1))
                {
                    ast[i].right_ += 1;
                }
            }

            for (int i = 1 + left.size(); i != ast.size(); ++i)
            {
                if (ast[i].left_ != static_cast<std::size_t>(-1))
                {
                    ast[i].left_ += 1 + left.size();
                }

                if (ast[i].right_ != static_cast<std::size_t>(-1))
                {
                    ast[i].right_ += 1 + left.size();
                }
            }

            return ast;
        }

        std::size_t size() const
        {
            return size_;
        }

    private:
        LHS const& lhs;
        RHS const& rhs;
        std::size_t size_;
        ct::frontend::Operation op;
    };

    // User Facing ct::vector class
    class vector
    {
        template <typename LHS, typename RHS>
        friend class VecExpression;

    public:
        explicit vector(std::size_t size)
          : id_(ct::frontend::get_vector_id())
          , size_(size)
          , node_(id_, ct::frontend::Operation::init_random,
                ct::frontend::Type::vector, size_)
        {
            ct::frontend::ASTQueue& queue =
                CT_ACCESS_SINGLETON(ct::frontend::ast_queue);

            queue.insert(node_);
        }

        explicit vector(std::size_t size, double value)
          : id_(ct::frontend::get_vector_id())
          , size_(size)
          , node_(id_, ct::frontend::Operation::init_value, value,
                ct::frontend::Type::vector, size_)
        {
            ct::frontend::ASTQueue& queue =
                CT_ACCESS_SINGLETON(ct::frontend::ast_queue);

            queue.insert(node_);
        }

        template <typename Expression>
        vector(Expression const& e)
          : id_(ct::frontend::get_vector_id())
        {
            std::vector<ct::frontend::ASTNode> instr = e();
            ct::frontend::ASTNode& root = instr.front();

            root.name_ = id_;
            node_ = ct::frontend::ASTNode{root};
            size_ = std::get<std::size_t>(root.dimensions_);

            ct::frontend::ASTQueue& queue =
                CT_ACCESS_SINGLETON(ct::frontend::ast_queue);
            queue.insert(instr);
        }

        std::size_t size() const
        {
            return size_;
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
        std::size_t size_;

        // AST Node Reference
        ct::frontend::ASTNode node_;
    };
}    // namespace ct
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
#include <iostream>
#include <memory>
#include <variant>
#include <vector>

#include "charm++.h"

namespace ct {
    namespace frontend {

        template <typename T, typename SingletonClass>
        class singleton
        {
        public:
            using value_type = T;
            using class_type = SingletonClass;

            // Non-copyable, non-movable
            singleton(singleton const&) = delete;
            singleton(singleton&&) = delete;
            singleton& operator=(singleton const&) = delete;
            singleton& operator=(singleton&&) = delete;

            static const std::unique_ptr<value_type>& instance()
            {
                static std::unique_ptr<value_type> inst{new value_type()};

                return inst;
            }

        protected:
            singleton() = default;
        };

#define CT_GENERATE_SINGLETON(type, name)                                      \
    class name : public singleton<type, name>                                  \
    {                                                                          \
    private:                                                                   \
        name() = default;                                                      \
    }

#define CT_ACCESS_SINGLETON(name) (*name::instance())

        enum class Operation : short
        {
            // Leaf Nodes
            noop = 0,
            init_random = 1,
            init_value = 2,
            copy = 3,

            // Middle/Head Nodes
            add = 10,
            sub = 11,
            mul = 12,
            div = 13,
            matmul = 14,
            axpy = 15,
            axpy_multiplier = 16
        };

        enum class Type : short
        {
            scalar = 0,
            vector = 1,
            matrix = 2,
            tensor = 3
        };

        struct ASTNode
        {
            std::size_t name_;
            Operation operation_;
            std::size_t copy_id_;
            double value_;
            Type type_;

            std::size_t vec_dim_;
            std::pair<std::size_t, std::size_t> mat_dim_;

            std::size_t left_;
            std::size_t right_;

            ASTNode() = default;

            void pup(PUP::er& p)
            {
                p | name_;
                p | operation_;
                p | copy_id_;
                p | value_;
                p | type_;
                p | vec_dim_;
                p | mat_dim_;
                p | left_;
                p | right_;
            }

            // Scalar constructor
            explicit ASTNode(std::size_t name, Operation op, Type t)
              : name_(name)
              , operation_(op)
              , copy_id_(-1)
              , value_(0)
              , type_(t)
              , vec_dim_(-1)
              , mat_dim_({-1, -1})
              , left_(-1)
              , right_(-1)
            {
            }

            // Scalar constructor
            explicit ASTNode(
                std::size_t name, Operation op, Type t, double value)
              : name_(name)
              , operation_(op)
              , copy_id_(-1)
              , value_(value)
              , type_(t)
              , vec_dim_(-1)
              , mat_dim_({-1, -1})
              , left_(-1)
              , right_(-1)
            {
            }

            // Vector constructor
            explicit ASTNode(
                std::size_t name, Operation op, Type t, std::size_t size)
              : name_(name)
              , operation_(op)
              , copy_id_(-1)
              , value_(0)
              , type_(t)
              , vec_dim_(size)
              , mat_dim_({-1, -1})
              , left_(-1)
              , right_(-1)
            {
            }

            explicit ASTNode(std::size_t name, Operation op, double value,
                Type t, std::size_t size)
              : name_(name)
              , operation_(op)
              , copy_id_(-1)
              , value_(value)
              , type_(t)
              , vec_dim_(size)
              , mat_dim_({-1, -1})
              , left_(-1)
              , right_(-1)
            {
            }

            explicit ASTNode(
                std::size_t name, Operation op, ASTNode const& node)
              : name_(name)
              , operation_(op)
              , copy_id_(node.name_)
              , value_(0)
              , type_(node.type_)
              , vec_dim_(node.vec_dim_)
              , mat_dim_(node.mat_dim_)
              , left_(-1)
              , right_(-1)
            {
            }

            explicit ASTNode(std::size_t name, Operation op, Type t,
                std::size_t rows, std::size_t cols)
              : name_(name)
              , operation_(op)
              , copy_id_(-1)
              , value_(0)
              , type_(t)
              , vec_dim_(-1)
              , mat_dim_({rows, cols})
              , left_(-1)
              , right_(-1)
            {
            }

            explicit ASTNode(std::size_t name, Operation op, double value,
                Type t, std::size_t rows, std::size_t cols)
              : name_(name)
              , operation_(op)
              , copy_id_(-1)
              , value_(value)
              , type_(t)
              , vec_dim_(-1)
              , mat_dim_({rows, cols})
              , left_(-1)
              , right_(-1)
            {
            }

            ASTNode(ASTNode const& other) = default;
            ASTNode(ASTNode&& other) = default;
            ASTNode& operator=(ASTNode const& other) = default;
            ASTNode& operator=(ASTNode&& other) = default;
        };

        bool is_init_type(ct::frontend::Operation op)
        {
            if (op == ct::frontend::Operation::noop)
                return true;

            if (op == ct::frontend::Operation::init_value)
                return true;

            if (op == ct::frontend::Operation::init_random)
                return true;

            if (op == ct::frontend::Operation::copy)
                return true;

            return false;
        }

        void parse_ast(std::vector<ASTNode> const& instr, std::size_t index)
        {
            if (index == 0 && !is_init_type(instr[index].operation_))
                ckout << instr[index].name_ << " = ";

            switch (instr[index].operation_)
            {
            case Operation::init_random:
                ckout << instr[index].name_ << " = [RANDOM]";
                return;
            case Operation::init_value:
                ckout << instr[index].name_ << " = VALUE["
                      << instr[index].value_ << "]";
                return;
            case Operation::noop:
                ckout << instr[index].name_;
                return;
            case Operation::copy:
                ckout << instr[index].name_ << " = " << instr[index].copy_id_;
                return;
            case Operation::add:
                parse_ast(instr, instr[index].left_);
                ckout << " + ";
                parse_ast(instr, instr[index].right_);
                return;
            case Operation::sub:
                parse_ast(instr, instr[index].left_);
                ckout << " - ";
                parse_ast(instr, instr[index].right_);
                return;
            }
        }

        class ASTQueue
        {
        public:
            using AST = std::vector<ASTNode>;

            ASTQueue() = default;

            void insert(ASTNode const& node)
            {
                ast_q.emplace_back(AST{node});
            }

            void insert(AST const& node)
            {
                ast_q.emplace_back(node);
            }

            void print_instructions() const
            {
                ckout << "Printing Instructions:" << endl;

                for (int i = 0; i != ast_q.size(); ++i)
                {
                    ckout << "Instruction " << i << ": ";
                    parse_ast(ast_q[i], 0);
                    ckout << endl;
                }
            }

            std::vector<AST> dispatch()
            {
                std::vector<AST> instr_list = ast_q;
                ast_q.clear();
                ++sdag_index_;

                return instr_list;
            }

            int sdag_index() const
            {
                return sdag_index_;
            }

            void pup(PUP::er& p)
            {
                p | ast_q;
                p | sdag_index_;
            }

        private:
            std::vector<AST> ast_q;
            int sdag_index_ = 0;
        };

        CT_GENERATE_SINGLETON(ASTQueue, ast_queue);

        CT_GENERATE_SINGLETON(std::size_t, vector_id);

        std::size_t get_vector_id()
        {
            std::size_t& id = CT_ACCESS_SINGLETON(vector_id);
            std::size_t curr_id = id++;

            return curr_id;
        }

        CT_GENERATE_SINGLETON(std::size_t, matrix_id);

        std::size_t get_matrix_id()
        {
            std::size_t& id = CT_ACCESS_SINGLETON(matrix_id);
            std::size_t curr_id = id++;

            return curr_id;
        }

        CT_GENERATE_SINGLETON(std::size_t, scalar_id);

        std::size_t get_scalar_id()
        {
            std::size_t& id = CT_ACCESS_SINGLETON(scalar_id);
            std::size_t curr_id = id++;

            return curr_id;
        }

    }    // namespace frontend

    template <typename LHS, typename RHS>
    class Expression
    {
    public:
        // Vector Expression generator
        Expression(LHS const& lhs_, RHS const& rhs_, std::size_t size,
            ct::frontend::Operation op_)
          : lhs(lhs_)
          , rhs(rhs_)
          , size_(size)
          , mat_dim_({-1, -1})
          , op(op_)
        {
        }

        // Matrix Expression generator
        Expression(LHS const& lhs_, RHS const& rhs_, std::size_t row,
            std::size_t cols, ct::frontend::Operation op_)
          : lhs(lhs_)
          , rhs(rhs_)
          , size_(-1)
          , mat_dim_(std::make_pair(row, cols))
          , op(op_)
        {
        }

        // Scalar Expression generator
        Expression(
            LHS const& lhs_, RHS const& rhs_, ct::frontend::Operation op_)
          : lhs(lhs_)
          , rhs(rhs_)
          , size_(-1)
          , mat_dim_({-1, -1})
          , op(op_)
        {
        }

        std::vector<ct::frontend::ASTNode> operator()() const
        {
            std::vector<ct::frontend::ASTNode> left = lhs();
            std::vector<ct::frontend::ASTNode> right = rhs();

            ct::frontend::ASTNode node;

            // Scalar node addition
            if (size_ == -1 && mat_dim_.first == -1)
                node = ct::frontend::ASTNode{static_cast<std::size_t>(-1), op,
                    ct::frontend::Type::scalar};
            // Matrix node addition
            else if (size_ == -1)
                node = ct::frontend::ASTNode{static_cast<std::size_t>(-1), op,
                    ct::frontend::Type::matrix, mat_dim_.first,
                    mat_dim_.second};
            else
                node = ct::frontend::ASTNode{static_cast<std::size_t>(-1), op,
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
        std::pair<std::size_t, std::size_t> mat_dim_;
        ct::frontend::Operation op;
    };

    namespace traits {

        template <typename T>
        struct isArrayTypeImpl
        {
        };

    }    // namespace traits

}    // namespace ct
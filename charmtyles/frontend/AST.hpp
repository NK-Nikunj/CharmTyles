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

namespace ct { namespace frontend {

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
        double value_;
        Type type_;

        std::variant<std::size_t, std::pair<std::size_t, std::size_t>>
            dimensions_;

        std::size_t left_;
        std::size_t right_;

        ASTNode() = default;

        explicit ASTNode(
            std::size_t name, Operation op, Type t, std::size_t size)
          : name_(name)
          , operation_(op)
          , value_(0)
          , type_(t)
          , dimensions_(size)
          , left_(-1)
          , right_(-1)
        {
        }

        explicit ASTNode(std::size_t name, Operation op, double value, Type t,
            std::size_t size)
          : name_(name)
          , operation_(op)
          , value_(value)
          , type_(t)
          , dimensions_(size)
          , left_(-1)
          , right_(-1)
        {
        }

        explicit ASTNode(std::size_t name, Operation op, Type t,
            std::size_t rows, std::size_t cols)
          : name_(name)
          , operation_(op)
          , value_(0)
          , type_(t)
          , dimensions_(std::make_pair(rows, cols))
          , left_(-1)
          , right_(-1)
        {
        }

        ASTNode(ASTNode const& other) = default;
        ASTNode(ASTNode&& other) = default;
        ASTNode& operator=(ASTNode const& other) = default;
        ASTNode& operator=(ASTNode&& other) = default;
    };

    class ASTQueue
    {
        bool is_init_type(ct::frontend::Operation op) const
        {
            if (op == ct::frontend::Operation::noop)
                return true;

            if (op == ct::frontend::Operation::init_value)
                return true;

            if (op == ct::frontend::Operation::init_random)
                return true;

            return false;
        }

        void parse_ast(
            std::vector<ASTNode> const& instr, std::size_t index) const
        {
            if (index == 0 && !is_init_type(instr[index].operation_))
                std::cout << instr[index].name_ << " = ";

            switch (instr[index].operation_)
            {
            case Operation::init_random:
                std::cout << instr[index].name_ << " = [RANDOM]";
                return;
            case Operation::init_value:
                std::cout << instr[index].name_ << " = VALUE["
                          << instr[index].value_ << "]";
                return;
            case Operation::noop:
                std::cout << instr[index].name_;
                return;
            case Operation::add:
                parse_ast(instr, instr[index].left_);
                std::cout << " + ";
                parse_ast(instr, instr[index].right_);
                return;
            case Operation::sub:
                parse_ast(instr, instr[index].left_);
                std::cout << "-";
                parse_ast(instr, instr[index].right_);
                return;
            }
        }

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
            std::cout << "Printing Instructions:" << std::endl;

            for (int i = 0; i != ast_q.size(); ++i)
            {
                std::cout << "Instruction " << i << ": ";
                parse_ast(ast_q[i], 0);
                std::cout << std::endl;
            }
        }

    private:
        std::vector<AST> ast_q;
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

}}    // namespace ct::frontend
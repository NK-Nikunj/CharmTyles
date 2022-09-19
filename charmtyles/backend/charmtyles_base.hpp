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

#include <charmtyles/backend/libcharmtyles.decl.h>

#include <eigen3/Eigen/Core>

/* readonly */ CProxy_charmtyles_base ct_proxy;

class set_future : public CBase_set_future
{
public:
    set_future() = default;

    void mark_complete(ck::future<bool> is_done)
    {
        is_done.set(true);
    }
};

class charmtyles_base : public CBase_charmtyles_base
{
    double execute_ast_vector_index(
        std::vector<ct::frontend::ASTNode> const& instruction,
        std::size_t curr_index, std::size_t iter_index)
    {
        const ct::frontend::ASTNode& node = instruction[curr_index];
        switch (node.operation_)
        {
        case ct::frontend::Operation::noop:
            return vec_map[node.name_][iter_index];

        case ct::frontend::Operation::add:
            return execute_ast_vector_index(
                       instruction, node.left_, iter_index) +
                execute_ast_vector_index(instruction, node.right_, iter_index);

        case ::ct::frontend::Operation::sub:
            return execute_ast_vector_index(
                       instruction, node.left_, iter_index) -
                execute_ast_vector_index(instruction, node.right_, iter_index);
        }
    }

    std::size_t get_vec_dim(std::size_t total_size)
    {
        if (total_size % num_partitions == 0)
            return total_size / num_partitions;

        if (thisIndex != num_partitions - 1)
            return total_size / (num_partitions + 1);

        return total_size % (num_partitions + 1);
    }

    void print_instructions(
        std::vector<std::vector<ct::frontend::ASTNode>> const& instr_list)
    {
        ckout << "Printing Instruction from chare array" << endl;
        int counter = 0;
        for (std::vector<ct::frontend::ASTNode> const& ast : instr_list)
        {
            ckout << "Instruction: " << counter++ << ": ";
            parse_ast(ast, 0);
            ckout << endl;
        }
    }

    void update_partitions(
        std::vector<std::vector<ct::frontend::ASTNode>> const& instr_list)
    {
        for (std::vector<ct::frontend::ASTNode> const& ast : instr_list)
        {
            execute_instruction(ast);
        }
    }

    void execute_instruction(
        std::vector<ct::frontend::ASTNode> const& instruction,
        std::size_t index = 0)
    {
        ct::frontend::ASTNode const& node = instruction[index];
        std::size_t node_id = node.name_;

        switch (instruction[index].operation_)
        {
        case ct::frontend::Operation::init_random:

            if (node.type_ == ct::frontend::Type::vector)
            {
                CkAssert((vec_map.size() + 1 == node_id) &&
                    "A vector is initialized before a dependent vector "
                    "initialization.");

                std::size_t vec_dim = get_vec_dim(node.vec_dim_);
                // TODO: Do random initialization here
                vec_map.emplace_back(std::vector<double>(vec_dim, 0));
            }

            return;

        case ct::frontend::Operation::init_value:

            if (node.type_ == ct::frontend::Type::vector)
            {
                CkAssert((vec_map.size() + 1 == node_id) &&
                    "A vector is initialized before a dependent vector "
                    "initialization.");

                std::size_t vec_dim = get_vec_dim(node.vec_dim_);
                // TODO: Do random initialization here
                vec_map.emplace_back(std::vector<double>(vec_dim, node.value_));
            }

            return;

        case ct::frontend::Operation::sub:
        case ct::frontend::Operation::add:

            if (node.type_ == ct::frontend::Type::vector)
            {
                // Is this a new node?
                if (node_id == vec_map.size())
                {
                    std::size_t vec_dim = get_vec_dim(node.vec_dim_);
                    // Create a new node first
                    vec_map.emplace_back(std::vector<double>(vec_dim));
                }

                // Main addition kernel
                for (std::size_t i = 0; i != vec_map[node_id].size(); ++i)
                {
                    vec_map[node_id][i] =
                        execute_ast_vector_index(instruction, 0, i);
                }
            }

            return;
        }
    }

public:
    charmtyles_base_SDAG_CODE;

    charmtyles_base(int num_chares, CProxy_set_future proxy)
      : num_partitions(num_chares)
      , SDAG_INDEX(0)
    {
        set_future_proxy = proxy;
        thisProxy[thisIndex].main_kernel();
    }

private:
    int num_partitions;
    std::vector<std::vector<double>> vec_map;

    int SDAG_INDEX;
    CProxy_set_future set_future_proxy;
};
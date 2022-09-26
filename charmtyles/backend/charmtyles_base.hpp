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

#include <charmtyles/util/view_vector.hpp>

#include <eigen3/Eigen/Core>

/* readonly */ CProxy_charmtyles_base ct_proxy;
/* readonly */ CProxy_set_future sf_proxy;

class set_future : public CBase_set_future
{
public:
    set_future() = default;

    void mark_complete()
    {
        is_done.set(true);
    }

    void put_future(ck::future<bool> is_done_)
    {
        is_done = is_done_;
    }

private:
    ck::future<bool> is_done;
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

        case ct::frontend::Operation::sub:
            return execute_ast_vector_index(
                       instruction, node.left_, iter_index) -
                execute_ast_vector_index(instruction, node.right_, iter_index);
        }

        return 0.;
    }

    double execute_ast_matrix_index(
        std::vector<ct::frontend::ASTNode> const& instruction,
        std::size_t curr_index, std::size_t iter_row, std::size_t iter_col)
    {
        const ct::frontend::ASTNode& node = instruction[curr_index];
        switch (node.operation_)
        {
        case ct::frontend::Operation::noop:
            return vec_map[node.name_](iter_row, iter_col);

        case ct::frontend::Operation::add:
            return execute_ast_matrix_index(
                       instruction, node.left_, iter_row, iter_col) +
                execute_ast_matrix_index(
                    instruction, node.right_, iter_row, iter_col);

        case ct::frontend::Operation::sub:
            return execute_ast_matrix_index(
                       instruction, node.left_, iter_row, iter_col) -
                execute_ast_matrix_index(
                    instruction, node.right_, iter_row, iter_col);
        }

        return 0.;
    }

    // Vector Dimensions
    std::size_t get_vec_dim(std::size_t total_size)
    {
        if (total_size % num_partitions == 0)
            return total_size / num_partitions;

        std::size_t stored_elems = total_size / num_partitions;
        std::size_t remainder_elems = total_size - stored_elems;

        if (thisIndex < remainder_elems)
            ++stored_elems;

        return stored_elems;
    }

    // Matrix Dimensions
    // square blocks on each chare
    // One possibility: fix the size of each block
    // 1024x1024 sized blocks
    // Padding? Followed by optimization
    // NumPEs => 6
    //
    // Chare array (1D) => NumPes 6
    // Partitioning array/matrix => NumPes

    // 4kx4k => 2D chare array 16 chares
    // different chare arrays for partitions
    // mxn nxk => They'll have different partitions

    // Block Size: Vector => 1024; Matrix => 1024x1024
    // Block Size: Vector => 256; Matrix => 1024x1024
    // sqrt(2d chare arrays size) = 1d chare array size

    std::pair<std::size_t, std::size_t> get_mat_dim(
        std::pair<std::size_t, std::size_t> mat_dims)
    {
        std::size_t rows = mat_dims.first;
        std::size_t cols = mat_dims.second;
        bool is_rows_fitted = false;
        bool is_cols_fitted = false;

        if (rows % num_partitions == 0)
        {
            rows = rows / num_partitions;
            is_rows_fitted = true;
        }
        else if (thisIndex != rows)
            rows = rows / (num_partitions + 1);

        if (cols % num_partitions == 0)
        {
            cols = cols / num_partitions;
            is_cols_fitted = true;
        }
        else
            cols = cols / (num_partitions + 1);

        // if (!is_rows_fitted && thisIndex)
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
                vec_map.emplace_back(Eigen::VectorXd::Random(vec_dim));
            }
            else if (node.type_ == ct::frontend::Type::matrix)
            {
                CkAssert((mat_map.size() + 1 == node_id) &&
                    "A vector is initialized before a dependent vector "
                    "initialization.");

                std::pair<std::size_t, std::size_t> mat_dim =
                    get_mat_dim(node.mat_dim_);
                // TODO: Do random initialization here
                vec_map.emplace_back(
                    Eigen::VectorXd::Random(mat_dim.first, mat_dim.second));
            }

            return;

        case ct::frontend::Operation::init_value:

            if (node.type_ == ct::frontend::Type::vector)
            {
                CkAssert((vec_map.size() + 1 == node_id) &&
                    "A vector is initialized before a dependent vector "
                    "initialization.");

                std::size_t vec_dim = get_vec_dim(node.vec_dim_);

                vec_map.emplace_back(
                    Eigen::VectorXd::Constant(vec_dim, node.value_));
            }
            else if (node.type_ == ct::frontend::Type::matrix)
            {
                CkAssert((mat_map.size() + 1 == node_id) &&
                    "A vector is initialized before a dependent vector "
                    "initialization.");

                std::pair<std::size_t, std::size_t> mat_dim =
                    get_mat_dim(node.mat_dim_);
                // TODO: Do random initialization here
                vec_map.emplace_back(
                    Eigen::VectorXd::Random(mat_dim.first, mat_dim.second));
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
                    vec_map.emplace_back(Eigen::VectorXd(vec_dim));
                }

                // Main addition kernel
                // TODO: Find a way to use eigen instead
                // TODO: VECTORIZE this loop!!
                // vec_map[node_id] = execute_ast_vector(instruction, 0);
                // Flatten the above AST to linear => Most important
                //      postorder traversal => create temp
                // Go through the AST and generate an Expression?
                // a = b + c - d
                // temp = c - d (EigenOp<>{c - d} => Eigen::matrix => construct c - d)
                // a = b + temp
                // linearized vector instruction
                // generate separate loop for each and let compiler fuse them

                for (std::size_t i = 0; i != vec_map[node_id].size(); ++i)
                {
                    vec_map[node_id][i] =
                        execute_ast_vector_index(instruction, 0, i);
                }
            }
            else if (node.type_ == ct::frontend::Type::matrix)
            {
                // Is this a new node?
                if (node_id == mat_map.size())
                {
                    std::pair<std::size_t, std::size_t> mat_dim =
                        get_mat_dim(node.mat_dim_);
                    // Create a new node first
                    vec_map.emplace_back(
                        Eigen::VectorXd(mat_dim.first, mat_dim.second));
                }

                // Main addition kernel
                // TODO: Find a way to use eigen instead
                // TODO: VECTORIZE this loop!!
                for (std::size_t row = 0; row != mat_map[node_id].rows(); ++row)
                {
                    for (std::size_t col = 0; col != mat_map[node_id].cols();
                         ++col)
                    {
                        mat_map[node_id](row, col) =
                            execute_ast_matrix_index(instruction, 0, row, col);
                    }
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
        vec_map.reserve(1000);
        mat_map.reserve(1000);
        set_future_proxy = proxy;
        thisProxy[thisIndex].main_kernel();
    }

private:
    int num_partitions;
    std::vector<Eigen::VectorXd> vec_map;
    std::vector<Eigen::MatrixXd> mat_map;

    int SDAG_INDEX;
    CProxy_set_future set_future_proxy;
};
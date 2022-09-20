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

#include "charm++.h"

namespace ct {

    template <typename T>
    class view_vector
    {
        class iterator
        {
        public:
            iterator(T* p)
              : _curr(p)
            {
            }

            iterator& operator++()
            {
                _curr++;
                return *this;
            }

            iterator& operator--()
            {
                _curr--;
                return *this;
            }

            T& operator*()
            {
                return *_curr;
            }

            bool operator==(const iterator& b) const
            {
                return *_curr == *b._curr;
            }

            bool operator!=(const iterator& b) const
            {
                return *_curr != *b._curr;
            }

        private:
            T* _curr;
        };

    public:
        view_vector()
          : data_(nullptr)
          , size_(0)
        {
        }

        explicit view_vector(std::size_t size)
          : data_(new T[size])
          , size_(size)
        {
            ckout << "View vector with size: " << size_ << endl;
        }

        explicit view_vector(std::size_t size, T value)
          : data_(new T[size])
          , size_(size)
        {
            for (std::size_t i = 0; i != size; ++i)
                data_[i] = value;
        }

        view_vector(view_vector const& other)
          : data_(new T[other.size_])
          , size_(other.size_)
        {
            // ckout << "Copy constructor called" << endl;
            // ckout << "Size: " << other.size_ << endl;
            std::copy(other.data_, other.data_ + other.size_, data_);
        }

        view_vector(view_vector&& other)
          : data_(other.data_)
          , size_(other.size_)
        {
            ckout << "Move constructor called" << endl;
            ckout << "Size: " << other.size_ << endl;
            other.data_ = nullptr;
        }

        view_vector& operator=(view_vector const& other)
        {
            if (this != &other)
                std::copy(other.data_, other.data_ + other.size_, data_);

            return *this;
        }

        view_vector& operator=(view_vector&& other)
        {
            T* temp = data_;

            data_ = other.data_;
            other.data_ = nullptr;

            delete[] temp;

            return *this;
        }

        ~view_vector()
        {
            delete[] data_;
        }

        T* data()
        {
            return data_;
        }

        const T* data() const
        {
            return data_;
        }

        iterator begin()
        {
            return iterator(&data_[0]);
        }

        const iterator begin() const
        {
            return iterator(&data_[0]);
        }

        iterator end()
        {
            return iterator(&data_[size_]);
        }

        const iterator end() const
        {
            return iterator(&data_[size_]);
        }

        const iterator cbegin() const
        {
            return iterator(&data_[0]);
        }

        const iterator cend() const
        {
            return iterator(&data_[size_]);
        }

        std::size_t size() const
        {
            return size_;
        }

        T& operator[](std::size_t idx)
        {
            return data_[idx];
        }

        const T& operator[](std::size_t idx) const
        {
            return data_[idx];
        }

        void pup(PUP::er& p)
        {
            p | size_;

            if (p.isUnpacking())
            {
                data_ = new T[size_];
            }

            for (int i = 0; i != size_; ++i)
                p | data_[i];
        }

    private:
        T* data_;
        std::size_t size_;
    };
}    // namespace ct
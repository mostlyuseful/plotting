#ifndef CANDIDATE_HPP
#define CANDIDATE_HPP

#include "path.hpp"

#include <memory>

struct Candidate {
    double dst;
    std::shared_ptr<Path> p;
    std::shared_ptr<Path> q;
    std::shared_ptr<Path> q_orig;
};

Candidate make_candidate(std::shared_ptr<Path> p,
                                std::shared_ptr<Path> q,
                                std::shared_ptr<Path> q_orig);

#endif // CANDIDATE_HPP

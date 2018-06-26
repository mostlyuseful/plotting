#include "candidate.hpp"

Candidate make_candidate(std::shared_ptr<Path> p, std::shared_ptr<Path> q,
                         std::shared_ptr<Path> q_orig) {
  auto const dst = (p->end() - q->start()).matrix().norm();
  return {dst, p, q, q_orig};
}

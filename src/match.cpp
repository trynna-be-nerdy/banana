#include <iostream>
#include <vector>
#include <string>
#include <memory>

class Match {
private:
  std::string m_name;
  int m_score;

public:
  Match(const std::string& name, int score)
    : m_name(name), m_score(score) {}

  ~Match() = default;

  const std::string& getName() const { return m_name; }
  int getScore() const { return m_score; }

  void setScore(int score) { m_score = score; }

  void display() const {
    std::cout << "Match: " << m_name << " | Score: " << m_score << std::endl;
  }
};

int main() {
  std::vector<std::unique_ptr<Match>> matches;

  matches.push_back(std::make_unique<Match>("Game 1", 100));
  matches.push_back(std::make_unique<Match>("Game 2", 250));

  for (const auto& match : matches) {
    match->display();
  }

  return 0;
}
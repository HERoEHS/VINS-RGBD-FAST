#include "VocabularyBinary.hpp"
#include <opencv2/core/core.hpp>
#include <stdexcept>
using namespace std;

VINSLoop::Vocabulary::Vocabulary()
: nNodes(0), nodes(nullptr), nWords(0), words(nullptr) {
}

VINSLoop::Vocabulary::~Vocabulary() {
    if (nodes != nullptr) {
        delete [] nodes;
        nodes = nullptr;
    }
    
    if (words != nullptr) {
        delete [] words;
        words = nullptr;
    }
}
    
void VINSLoop::Vocabulary::serialize(ofstream& stream) {
    stream.write((const char *)this, staticDataSize());
    stream.write((const char *)nodes, sizeof(Node) * nNodes);
    stream.write((const char *)words, sizeof(Word) * nWords);
}
    
void VINSLoop::Vocabulary::deserialize(ifstream& stream) {
    if (!stream.is_open()) {
        throw runtime_error("Cannot open binary vocabulary file");
    }

    if (!stream.read((char *)this, staticDataSize())) {
        throw runtime_error("Cannot read binary vocabulary header");
    }

    if (nNodes < 0 || nWords < 0) {
        throw runtime_error("Binary vocabulary contains invalid item counts");
    }

    const streampos payload_begin = stream.tellg();
    stream.seekg(0, ios::end);
    const streampos file_end = stream.tellg();
    stream.seekg(payload_begin);

    const uint64_t expected_size = static_cast<uint64_t>(staticDataSize())
        + static_cast<uint64_t>(sizeof(Node)) * static_cast<uint64_t>(nNodes)
        + static_cast<uint64_t>(sizeof(Word)) * static_cast<uint64_t>(nWords);
    if (file_end < 0 || static_cast<uint64_t>(file_end) < expected_size) {
        throw runtime_error("Binary vocabulary file is truncated or corrupt");
    }
    
    nodes = new Node[nNodes];
    if (!stream.read((char *)nodes, sizeof(Node) * nNodes)) {
        throw runtime_error("Cannot read binary vocabulary nodes");
    }
    
    words = new Word[nWords];
    if (!stream.read((char *)words, sizeof(Word) * nWords)) {
        throw runtime_error("Cannot read binary vocabulary words");
    }
}

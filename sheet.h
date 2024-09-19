#pragma once

#include "cell.h"
#include "common.h"

#include <functional>
#include <unordered_map>

class SheetHasher {
public:
	size_t operator()(const Position p) const {
		return p.col * 1'00000 + p.row;
	}
};

class SheetComparator {
public:
	bool operator()(const Position& lhs, const Position& rhs) const {
		return lhs == rhs;
	}
};

class Sheet : public SheetInterface {
public:
	~Sheet();

	void SetCell(Position pos, std::string text) override;

	const CellInterface* GetCell(Position pos) const override;
	CellInterface* GetCell(Position pos) override;

	void ClearCell(Position pos) override;

	Size GetPrintableSize() const override;

	void PrintValues(std::ostream& output) const override;
	void PrintTexts(std::ostream& output) const override;

	// Можете дополнить ваш класс нужными полями и методами

private:
	std::unordered_map<Position, Cell, SheetHasher, SheetComparator> sheet_;
	std::unordered_map<Position, std::vector<Position>, SheetHasher, SheetComparator> dependencies_;
};